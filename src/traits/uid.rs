//! # Hardware unique identifier trait
//!
//! Provides an abstraction for querying factory-programmed electronic serial
//! numbers (silicon UIDs) across different microcontroller architectures.
//!
//! The trait provides a fixed maximum 12-byte raw UID array (`MAX_UID_BYTES = 12`)
//! where shorter hardware IDs (such as 48-bit MAC addresses or 64-bit chip IDs)
//! have trailing bytes zero-padded. It also provides a zero-heap stack-based
//! canonical ASCII hexadecimal string formatter (`CANONICAL_HEX_LEN = 24`).

/// Maximum size in bytes of a hardware unique identifier (96 bits = 12 bytes).
pub const MAX_UID_BYTES: usize = 12;

/// Length in bytes of the canonical uppercase hexadecimal representation (24 characters).
pub const CANONICAL_HEX_LEN: usize = 24;

/// Hardware unique identifier trait for silicon electronic serial numbers.
///
/// Implementations provide access to factory-programmed hardware serial numbers
/// without dynamic memory allocation.
pub trait HardwareUid {
    /// Return the raw hardware UID bytes, with any unused trailing bytes zero-padded
    /// up to [`MAX_UID_BYTES`].
    fn uid_bytes(&self) -> [u8; MAX_UID_BYTES];

    /// Return the actual number of valid bytes in the hardware UID
    /// (e.g. 6 for ESP32 MAC, 8 for RP2040/RP2350, 12 for STM32/CH32).
    fn uid_len(&self) -> usize;

    /// Format the valid UID bytes as uppercase canonical ASCII hexadecimal characters ('0'-'9', 'A'-'F').
    ///
    /// Writes into caller-provided stack buffer `out` and returns the number of ASCII bytes written
    /// (`2 * uid_len()`).
    fn format_hex(&self, out: &mut [u8; CANONICAL_HEX_LEN]) -> usize {
        const HEX_CHARS: &[u8; 16] = b"0123456789ABCDEF";
        let bytes = self.uid_bytes();
        let len = self.uid_len().min(MAX_UID_BYTES);
        for i in 0..len {
            let b = bytes[i];
            out[2 * i] = HEX_CHARS[(b >> 4) as usize];
            out[2 * i + 1] = HEX_CHARS[(b & 0x0F) as usize];
        }
        2 * len
    }
}

impl<T: HardwareUid + ?Sized> HardwareUid for &T {
    fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
        (**self).uid_bytes()
    }

    fn uid_len(&self) -> usize {
        (**self).uid_len()
    }

    fn format_hex(&self, out: &mut [u8; CANONICAL_HEX_LEN]) -> usize {
        (**self).format_hex(out)
    }
}

/// A static, pre-buffered hardware unique identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StaticUid {
    bytes: [u8; MAX_UID_BYTES],
    len: usize,
}

impl StaticUid {
    /// Create a new `StaticUid` from a slice of bytes.
    ///
    /// Copies up to [`MAX_UID_BYTES`] bytes and zero-pads any remaining trailing bytes.
    pub const fn from_slice(slice: &[u8]) -> Self {
        let mut bytes = [0u8; MAX_UID_BYTES];
        let len = if slice.len() > MAX_UID_BYTES {
            MAX_UID_BYTES
        } else {
            slice.len()
        };
        let mut i = 0;
        while i < len {
            bytes[i] = slice[i];
            i += 1;
        }
        Self { bytes, len }
    }

    /// Create a new `StaticUid` from a 12-byte raw array (e.g. STM32 / CH32 96-bit UID).
    pub const fn from_raw_12(bytes: [u8; 12]) -> Self {
        Self { bytes, len: 12 }
    }

    /// Create a new `StaticUid` from an 8-byte array (e.g. RP2040/RP2350 64-bit UID), zero-padding to 12 bytes.
    pub const fn from_raw_8(b: [u8; 8]) -> Self {
        Self {
            bytes: [b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], 0, 0, 0, 0],
            len: 8,
        }
    }

    /// Create a new `StaticUid` from a 6-byte array (e.g. MAC address), zero-padding to 12 bytes.
    pub const fn from_raw_6(b: [u8; 6]) -> Self {
        Self {
            bytes: [b[0], b[1], b[2], b[3], b[4], b[5], 0, 0, 0, 0, 0, 0],
            len: 6,
        }
    }
}

impl HardwareUid for StaticUid {
    fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
        self.bytes
    }

    fn uid_len(&self) -> usize {
        self.len
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Mock12ByteUid;
    impl HardwareUid for Mock12ByteUid {
        fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
            [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x10, 0x20, 0x30, 0x40]
        }
        fn uid_len(&self) -> usize {
            12
        }
    }

    struct Mock8ByteUid;
    impl HardwareUid for Mock8ByteUid {
        fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
            [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22, 0, 0, 0, 0]
        }
        fn uid_len(&self) -> usize {
            8
        }
    }

    struct Mock6ByteUid;
    impl HardwareUid for Mock6ByteUid {
        fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
            [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0, 0, 0, 0, 0, 0]
        }
        fn uid_len(&self) -> usize {
            6
        }
    }

    #[test]
    fn test_mock_12_byte_formatting() {
        let uid = Mock12ByteUid;
        assert_eq!(uid.uid_len(), 12);
        let raw = uid.uid_bytes();
        assert_eq!(raw.len(), MAX_UID_BYTES);

        let mut hex_buf = [0u8; CANONICAL_HEX_LEN];
        let written = uid.format_hex(&mut hex_buf);
        assert_eq!(written, 24);
        assert_eq!(&hex_buf, b"0123456789ABCDEF10203040");
    }

    #[test]
    fn test_mock_8_byte_formatting_and_zero_padding() {
        let uid = Mock8ByteUid;
        assert_eq!(uid.uid_len(), 8);
        let raw = uid.uid_bytes();
        // Verify trailing bytes are zero padded
        assert_eq!(&raw[8..12], &[0, 0, 0, 0]);

        let mut hex_buf = [0u8; CANONICAL_HEX_LEN];
        let written = uid.format_hex(&mut hex_buf);
        assert_eq!(written, 16);
        assert_eq!(&hex_buf[..written], b"AABBCCDDEEFF1122");
    }

    #[test]
    fn test_mock_6_byte_formatting_and_zero_padding() {
        let uid = Mock6ByteUid;
        assert_eq!(uid.uid_len(), 6);
        let raw = uid.uid_bytes();
        // Verify trailing bytes are zero padded
        assert_eq!(&raw[6..12], &[0, 0, 0, 0, 0, 0]);

        let mut hex_buf = [0u8; CANONICAL_HEX_LEN];
        let written = uid.format_hex(&mut hex_buf);
        assert_eq!(written, 12);
        assert_eq!(&hex_buf[..written], b"123456789ABC");
    }

    #[test]
    fn test_static_uid_constructors() {
        let s12 = StaticUid::from_raw_12([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
        assert_eq!(s12.uid_len(), 12);
        assert_eq!(s12.uid_bytes()[11], 12);

        let s8 = StaticUid::from_raw_8([0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8]);
        assert_eq!(s8.uid_len(), 8);
        assert_eq!(&s8.uid_bytes()[8..12], &[0, 0, 0, 0]);

        let s6 = StaticUid::from_raw_6([0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE]);
        assert_eq!(s6.uid_len(), 6);
        assert_eq!(&s6.uid_bytes()[6..12], &[0, 0, 0, 0, 0, 0]);
        let mut buf = [0u8; CANONICAL_HEX_LEN];
        let written = s6.format_hex(&mut buf);
        assert_eq!(written, 12);
        assert_eq!(&buf[..written], b"DEADBEEFCAFE");

        let slice_uid = StaticUid::from_slice(&[0x11, 0x22]);
        assert_eq!(slice_uid.uid_len(), 2);
        assert_eq!(&slice_uid.uid_bytes()[..2], &[0x11, 0x22]);
        assert_eq!(&slice_uid.uid_bytes()[2..], &[0; 10]);
    }

    #[test]
    fn test_reference_dispatch() {
        let uid = StaticUid::from_raw_6([0x01, 0x23, 0x45, 0x67, 0x89, 0xAB]);
        let r: &dyn HardwareUid = &uid;
        assert_eq!(r.uid_len(), 6);
        let mut buf = [0u8; CANONICAL_HEX_LEN];
        let written = r.format_hex(&mut buf);
        assert_eq!(written, 12);
        assert_eq!(&buf[..written], b"0123456789AB");
    }
}
