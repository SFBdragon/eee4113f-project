//! Protocol implementation library shared between the controller and module.

// Don't link to the standard library `std` unless we're running tests.
#![cfg_attr(not(any(test, feature = "test-utils")), no_std)]

pub mod app;
pub mod byteutils;
pub mod lora;
pub mod phy;
pub mod rtt;
pub mod wifi;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct LoRaAddr(u16);

impl LoRaAddr {
    pub const RESERVED_BIT: u16 = 1 << (u16::BITS - 1);

    const BCAST: u16 = u16::MAX & !Self::RESERVED_BIT;

    pub const fn bcast() -> Self {
        Self::from_raw(Self::BCAST)
    }

    pub const fn from_raw(r: u16) -> Self {
        Self(r & !Self::RESERVED_BIT)
    }

    pub const fn to_raw(self) -> u16 {
        self.0
    }

    pub const fn is_bcast(self) -> bool {
        self.0 == Self::BCAST
    }

    pub const fn to_le_bytes(&self) -> [u8; size_of::<u16>()] {
        self.0.to_le_bytes()
    }

    pub fn from_str(text: &str) -> Result<Self, ()> {
        if text.len() != 5 {
            return Err(());
        }

        let lo = u8::from_str_radix(&text[0..2], 16).map_err(|_| ())?;
        let hi = u8::from_str_radix(&text[3..5], 16).map_err(|_| ())?;

        Ok(LoRaAddr::from_raw(u16::from_le_bytes([lo, hi])))
    }
}

impl core::fmt::Debug for LoRaAddr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("LoRaAddr({})", self))
    }
}

impl core::fmt::Display for LoRaAddr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let bytes = self.to_le_bytes();
        f.write_fmt(format_args!("{:02X}:{:02X}", bytes[0], bytes[1]))
    }
}

/// Caller provides this. Receives the full frame bytes *before* the CRC field.
/// Returns a 2-byte CRC in little-endian order packed into the low 16 bits.
pub type CrcFn = fn(data: *const u8, len: usize) -> u16;

fn call_crc_fn(f: CrcFn, buf: &[u8]) -> u16 {
    f(buf.as_ptr(), buf.len())
}
