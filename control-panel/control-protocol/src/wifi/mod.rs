pub mod common;
pub mod ping;
pub mod sender;

pub use sender::Sender;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Mac(u64);
impl Mac {
    pub fn bcast() -> Self {
        Self(0x0000FFFF_FFFFFFFF)
    }

    pub fn to_u64(&self) -> u64 {
        self.0
    }

    pub fn is_bcast(&self) -> bool {
        *self == Self::bcast()
    }
}
impl From<u64> for Mac {
    fn from(value: u64) -> Self {
        Mac(value & 0x0000FFFF_FFFFFFFF)
    }
}

impl core::fmt::Debug for Mac {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("Mac({})", self))
    }
}

impl core::fmt::Display for Mac {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let b = self.0.to_le_bytes();
        f.write_fmt(format_args!(
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            b[0], b[1], b[2], b[3], b[4], b[5]
        ))
    }
}
