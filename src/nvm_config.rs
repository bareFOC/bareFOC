//! # config — persistent calibration block (Phase E1)
//!
//! 64-byte, 16-byte aligned (H5 quadword program unit), CRC32-checked record
//! kept in the last 8 KB sector of flash bank 2 (`0x0801E000..0x08020000`,
//! offset `0x1E000` from the flash start). Bank 2 is never executed from in
//! this image (the app occupies bank 1), so erase/program never stalls the
//! control loop's instruction fetches.
//!
//! Persisted fields: the electrical zero angle and sensor direction produced
//! by the `c` commutation calibration, plus the I2C address (`i2c_addr`) and
//! the semantic joint slot (`joint_slot`) adopted per design D7, so a
//! power-cycled board can skip the rotating alignment step, keep its bus
//! address, and remember which joint it is.
//!
//! References: `examples/miniata_barefoc/progress.md` Phase E (E1), the
//! record layout follows `barefoc-kinematics::calibration::storage`
//! (`FlashCalibrationData`: magic + version + payload + IEEE CRC32 trailer).
//!
//! Schema:
//! - v1 (legacy): magic/version/flags (8 B), zero (4 B), sensor_dir (4 B),
//!   reserved (44 B), CRC (4 B) = 64 B. Loaded with defaults
//!   `i2c_addr = 0x30`, `joint_slot = 0`.
//! - v2: magic/version/flags (8 B), zero (4 B), sensor_dir (4 B),
//!   i2c_addr (1 B), joint_slot (1 B), reserved (42 B), CRC (4 B) = 64 B.

use embassy_stm32::flash::Flash;
use embassy_stm32::mode::Blocking;

/// Config record offset, as required by `Flash::blocking_*` — relative to the
/// flash start, derived from generated BSP NVM sector origin.
const CONFIG_OFFSET: u32 = crate::bsp::NVM_SECTOR_ORIGIN - crate::bsp::FLASH_ORIGIN;
/// Config sector length. Erase granularity.
const SECTOR_LEN: u32 = crate::bsp::NVM_SECTOR_SIZE;
/// Serialized record size (must be a multiple of the 16-byte program unit).
const RECORD_LEN: usize = 64;

/// Magic signature for the Miniata config block ("MINI").
pub const CONFIG_MAGIC: u32 = 0x4D49_4E49;
/// Config record schema version.
pub const CONFIG_VERSION: u16 = 2;

/// Legacy schema version still accepted by the loader (with defaults).
pub const CONFIG_VERSION_LEGACY: u16 = 1;

/// Default I2C address carried by v1 records.
pub const DEFAULT_I2C_ADDR: u8 = 0x30;

/// Persistent calibration record.
///
/// Layout (v2): magic + version + flags (8 B), payload (8 B: zero + sensor_dir),
/// identity (2 B: i2c_addr + joint_slot), reserved (42 B), CRC32 over the
/// preceding 60 bytes (4 B) = 64 B.
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlashConfigData {
    /// Magic identifier (`0x4D49_4E49`, "MINI").
    pub magic: u32,
    /// Schema format version.
    pub version: u16,
    /// Configuration feature flags (bit0 = calibration block populated).
    pub flags: u16,
    /// Electrical zero angle from commutation calibration (rad).
    pub zero_electrical_angle: f32,
    /// Sensor direction from commutation calibration (+1.0 / -1.0).
    pub sensor_dir: f32,
    /// Persisted I2C address (design D7 manual addressing).
    pub i2c_addr: u8,
    /// Persisted semantic joint slot (design D7).
    pub joint_slot: u8,
    /// Reserved for future expansion (bump `CONFIG_VERSION` to change layout).
    pub reserved: [u8; 42],
    /// CRC32 over bytes 0..60.
    pub checksum: u32,
}

impl FlashConfigData {
    /// Build a storeable record from calibrated values.
    pub const fn new(
        zero_electrical_angle: f32,
        sensor_dir: f32,
        i2c_addr: u8,
        joint_slot: u8,
    ) -> Self {
        Self {
            magic: CONFIG_MAGIC,
            version: CONFIG_VERSION,
            flags: 1,
            zero_electrical_angle,
            sensor_dir,
            i2c_addr,
            joint_slot,
            reserved: [0u8; 42],
            checksum: 0,
        }
    }

    /// IEEE 802.3 CRC32 over the first 60 bytes of the record.
    pub const fn compute_checksum(bytes: &[u8]) -> u32 {
        compute_bytes_crc32(bytes)
    }

    /// Re-derive and store the checksum from this record's serialization.
    pub fn update_checksum(&mut self) {
        let raw = self.to_bytes_raw();
        self.checksum = Self::compute_checksum(&raw[..60]);
    }

    /// Validate magic, version, and checksum against a raw 64-byte record.
    ///
    /// Accepts schema v1 (legacy) and v2. v1 records load with defaults
    /// `i2c_addr = 0x30`, `joint_slot = 0`; their reserved area is discarded.
    /// Unknown versions are rejected.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ()> {
        if bytes.len() < RECORD_LEN {
            return Err(());
        }
        let magic = u32::from_le_bytes(bytes[0..4].try_into().expect("fixed slice"));
        let version = u16::from_le_bytes(bytes[4..6].try_into().expect("fixed slice"));
        let flags = u16::from_le_bytes(bytes[6..8].try_into().expect("fixed slice"));
        let zero_electrical_angle =
            f32::from_le_bytes(bytes[8..12].try_into().expect("fixed slice"));
        let sensor_dir = f32::from_le_bytes(bytes[12..16].try_into().expect("fixed slice"));

        if magic != CONFIG_MAGIC {
            return Err(());
        }
        if version != CONFIG_VERSION_LEGACY && version != CONFIG_VERSION {
            return Err(());
        }

        let (i2c_addr, joint_slot, reserved) = if version == CONFIG_VERSION_LEGACY {
            (DEFAULT_I2C_ADDR, 0u8, [0u8; 42])
        } else {
            let i2c_addr = bytes[16];
            let joint_slot = bytes[17];
            let mut reserved = [0u8; 42];
            reserved.copy_from_slice(&bytes[18..60]);
            (i2c_addr, joint_slot, reserved)
        };

        let checksum = u32::from_le_bytes(bytes[60..64].try_into().expect("fixed slice"));

        let data = Self {
            magic,
            version,
            flags,
            zero_electrical_angle,
            sensor_dir,
            i2c_addr,
            joint_slot,
            reserved,
            checksum,
        };
        let expected = Self::compute_checksum(&bytes[..60]);
        if data.checksum != expected {
            return Err(());
        }
        Ok(data)
    }

    /// Serialize the record to exactly 64 bytes (checksum as stored).
    pub fn to_bytes_raw(&self) -> [u8; 64] {
        let mut buf = [0u8; RECORD_LEN];
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4..6].copy_from_slice(&self.version.to_le_bytes());
        buf[6..8].copy_from_slice(&self.flags.to_le_bytes());
        buf[8..12].copy_from_slice(&self.zero_electrical_angle.to_le_bytes());
        buf[12..16].copy_from_slice(&self.sensor_dir.to_le_bytes());
        buf[16] = self.i2c_addr;
        buf[17] = self.joint_slot;
        buf[18..60].copy_from_slice(&self.reserved);
        buf[60..64].copy_from_slice(&self.checksum.to_le_bytes());
        buf
    }

    /// Serialize with a refreshed checksum.
    pub fn to_bytes(&mut self) -> [u8; 64] {
        self.update_checksum();
        self.to_bytes_raw()
    }
}

/// IEEE 802.3 CRC32 over `data` (same polynomial table family as the
/// `barefoc-kinematics` flash storage module; duplicated here because that
/// helper is private to the crate and this example is standalone).
const fn compute_bytes_crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    let mut i = 0usize;
    while i < data.len() {
        crc ^= data[i] as u32;
        let mut bit = 0u32;
        while bit < 8 {
            crc = if (crc & 1) != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
            bit += 1;
        }
        i += 1;
    }
    !crc
}

/// Load the persisted calibration. Returns `None` when the sector is blank
/// (magic mismatch), from an unsupported schema version, or fails CRC — in
/// every such case the caller must treat calibration as unknown and run `c`.
pub fn load(flash: &mut Flash<'_, Blocking>) -> Option<FlashConfigData> {
    let mut buf = [0u8; RECORD_LEN];
    flash.blocking_read(CONFIG_OFFSET, &mut buf).ok()?;
    let data = FlashConfigData::from_bytes(&buf).ok()?;
    // CRC/magic/version already validated by `from_bytes`. A record with a
    // zero calibration is still a legitimate (identity-only) save, so we do
    // not gate on `sensor_dir`/`zero_electrical_angle` here; callers apply
    // the calibration fields conditionally.
    Some(data)
}

/// Erase the config sector and program the record (checksum refreshed).
///
/// Blocking call — under a relaxing flash driver this pays a full 8 KB sector
/// erase (~tens of ms) plus the program pulses; only call while disarmed.
pub fn store(flash: &mut Flash<'_, Blocking>, record: &mut FlashConfigData) -> Result<(), ()> {
    let bytes = record.to_bytes();
    flash
        .blocking_erase(CONFIG_OFFSET, CONFIG_OFFSET + SECTOR_LEN)
        .map_err(|_| ())?;
    flash.blocking_write(CONFIG_OFFSET, &bytes).map_err(|_| ())
}
