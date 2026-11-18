//! # calibration::storage — non-volatile flash configuration data structure
//!
//! Provides a 64-byte quadword-aligned configuration structure with CRC32 integrity
//! checking for non-volatile flash memory storage.

/// Magic signature for BareFOC kinematics configuration block ("BFOC").
pub const FLASH_MAGIC: u32 = 0x4246_4F43;

/// Current storage schema version.
pub const FLASH_VERSION: u16 = 1;

/// Storage errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageError {
    /// Invalid magic signature (uninitialized or corrupted flash).
    InvalidMagic,
    /// Schema version mismatch.
    UnsupportedVersion,
    /// CRC32 checksum mismatch.
    ChecksumMismatch,
    /// Buffer length insufficient.
    BufferTooSmall,
}

/// 64-byte quadword-aligned flash configuration header.
#[repr(C, align(64))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlashCalibrationData {
    /// Magic identifier (`0x4246_4F43`).
    pub magic: u32,
    /// Schema format version.
    pub version: u16,
    /// Configuration feature flags.
    pub flags: u16,
    /// Linear gear ratio (if applicable).
    pub gear_ratio: f32,
    /// Zero offset in motor radians.
    pub zero_offset: f32,
    /// Backlash half-deadband amplitude $\delta$ in radians.
    pub backlash_delta: f32,
    /// Torsional gear stiffness $K_{\text{gear}}$ in Nm/rad.
    pub torsional_stiffness: f32,
    /// Discovered minimum joint angle in radians.
    pub joint_min: f32,
    /// Discovered maximum joint angle in radians.
    pub joint_max: f32,
    /// Minimum motor angle for LUT in radians.
    pub lut_mot_min: f32,
    /// Maximum motor angle for LUT in radians.
    pub lut_mot_max: f32,
    /// Number of points in LUT table.
    pub lut_points: u16,
    /// Reserved padding bytes for future expansion and alignment.
    pub reserved: [u8; 18],
    /// CRC32 checksum over the preceding 60 bytes.
    pub checksum: u32,
}

impl Default for FlashCalibrationData {
    fn default() -> Self {
        let mut data = Self {
            magic: FLASH_MAGIC,
            version: FLASH_VERSION,
            flags: 0,
            gear_ratio: 1.0,
            zero_offset: 0.0,
            backlash_delta: 0.0,
            torsional_stiffness: 0.0,
            joint_min: 0.0,
            joint_max: 0.0,
            lut_mot_min: 0.0,
            lut_mot_max: 0.0,
            lut_points: 0,
            reserved: [0u8; 18],
            checksum: 0,
        };
        data.checksum = data.compute_checksum();
        data
    }
}

impl FlashCalibrationData {
    /// Compute standard CRC32 over the first 60 bytes of the structure.
    pub fn compute_checksum(&self) -> u32 {
        let bytes = self.to_bytes_raw();
        crc32(&bytes[..60])
    }

    /// Validate structure integrity (magic, version, and checksum).
    pub fn validate(&self) -> Result<(), StorageError> {
        if self.magic != FLASH_MAGIC {
            return Err(StorageError::InvalidMagic);
        }
        if self.version != FLASH_VERSION {
            return Err(StorageError::UnsupportedVersion);
        }
        if self.checksum != self.compute_checksum() {
            return Err(StorageError::ChecksumMismatch);
        }
        Ok(())
    }

    /// Update checksum in-place.
    pub fn update_checksum(&mut self) {
        self.checksum = self.compute_checksum();
    }

    /// Serialize structure into a fixed 64-byte array.
    pub fn to_bytes(&self) -> [u8; 64] {
        let mut data = *self;
        data.checksum = data.compute_checksum();
        data.to_bytes_raw()
    }

    fn to_bytes_raw(self) -> [u8; 64] {
        let mut buf = [0u8; 64];
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4..6].copy_from_slice(&self.version.to_le_bytes());
        buf[6..8].copy_from_slice(&self.flags.to_le_bytes());
        buf[8..12].copy_from_slice(&self.gear_ratio.to_le_bytes());
        buf[12..16].copy_from_slice(&self.zero_offset.to_le_bytes());
        buf[16..20].copy_from_slice(&self.backlash_delta.to_le_bytes());
        buf[20..24].copy_from_slice(&self.torsional_stiffness.to_le_bytes());
        buf[24..28].copy_from_slice(&self.joint_min.to_le_bytes());
        buf[28..32].copy_from_slice(&self.joint_max.to_le_bytes());
        buf[32..36].copy_from_slice(&self.lut_mot_min.to_le_bytes());
        buf[36..40].copy_from_slice(&self.lut_mot_max.to_le_bytes());
        buf[40..42].copy_from_slice(&self.lut_points.to_le_bytes());
        buf[42..60].copy_from_slice(&self.reserved);
        buf[60..64].copy_from_slice(&self.checksum.to_le_bytes());
        buf
    }

    /// Deserialize structure from a 64-byte slice and validate integrity.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, StorageError> {
        if bytes.len() < 64 {
            return Err(StorageError::BufferTooSmall);
        }
        let magic = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let version = u16::from_le_bytes(bytes[4..6].try_into().unwrap());
        let flags = u16::from_le_bytes(bytes[6..8].try_into().unwrap());
        let gear_ratio = f32::from_le_bytes(bytes[8..12].try_into().unwrap());
        let zero_offset = f32::from_le_bytes(bytes[12..16].try_into().unwrap());
        let backlash_delta = f32::from_le_bytes(bytes[16..20].try_into().unwrap());
        let torsional_stiffness = f32::from_le_bytes(bytes[20..24].try_into().unwrap());
        let joint_min = f32::from_le_bytes(bytes[24..28].try_into().unwrap());
        let joint_max = f32::from_le_bytes(bytes[28..32].try_into().unwrap());
        let lut_mot_min = f32::from_le_bytes(bytes[32..36].try_into().unwrap());
        let lut_mot_max = f32::from_le_bytes(bytes[36..40].try_into().unwrap());
        let lut_points = u16::from_le_bytes(bytes[40..42].try_into().unwrap());
        let mut reserved = [0u8; 18];
        reserved.copy_from_slice(&bytes[42..60]);
        let checksum = u32::from_le_bytes(bytes[60..64].try_into().unwrap());

        let data = Self {
            magic,
            version,
            flags,
            gear_ratio,
            zero_offset,
            backlash_delta,
            torsional_stiffness,
            joint_min,
            joint_max,
            lut_mot_min,
            lut_mot_max,
            lut_points,
            reserved,
            checksum,
        };

        data.validate()?;
        Ok(data)
    }

    /// Pack header along with an $N$-element LUT table into an output buffer.
    pub fn pack_with_lut<const N: usize>(
        &self,
        table: &[f32; N],
        out_buf: &mut [u8],
    ) -> Result<usize, StorageError> {
        let total_size = 64 + N * 4;
        if out_buf.len() < total_size {
            return Err(StorageError::BufferTooSmall);
        }
        let mut hdr = *self;
        hdr.lut_points = N as u16;
        hdr.update_checksum();
        out_buf[..64].copy_from_slice(&hdr.to_bytes());

        for (i, &val) in table.iter().enumerate() {
            let offset = 64 + i * 4;
            out_buf[offset..offset + 4].copy_from_slice(&val.to_le_bytes());
        }

        Ok(total_size)
    }

    /// Unpack header and $N$-element LUT table from a buffer.
    pub fn unpack_with_lut<const N: usize>(
        buf: &[u8],
    ) -> Result<(Self, [f32; N]), StorageError> {
        let total_size = 64 + N * 4;
        if buf.len() < total_size {
            return Err(StorageError::BufferTooSmall);
        }
        let hdr = Self::from_bytes(&buf[..64])?;
        let mut table = [0.0f32; N];
        for (i, slot) in table.iter_mut().enumerate() {
            let offset = 64 + i * 4;
            *slot = f32::from_le_bytes(buf[offset..offset + 4].try_into().unwrap());
        }
        Ok((hdr, table))
    }
}

/// Standard IEEE 802.3 CRC32 calculation.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = if (crc & 1) != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}
