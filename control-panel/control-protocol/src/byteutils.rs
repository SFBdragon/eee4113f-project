//! Custom binary parsing routines.
//!
//! Everything is little-endian.

#[macro_export]
macro_rules! first {
    ($t:ty, $buf:expr) => {{
        let (chunk, rem) = $buf.split_first_chunk::<{ size_of::<$t>() }>().unwrap();
        (rem, <$t>::from_le_bytes(*chunk))
    }};
}
pub(crate) use first;

#[macro_export]
macro_rules! last {
    ($t:ty, $buf:expr) => {{
        let (rem, chunk) = $buf.split_last_chunk::<{ size_of::<$t>() }>().unwrap();
        (rem, <$t>::from_le_bytes(*chunk))
    }};
}
pub(crate) use last;

#[macro_export]
macro_rules! write_int {
    ($pos:expr, $buf:expr, $val:expr) => {{
        let le_bytes = $val.to_le_bytes();
        let (target, _): (&mut [_], _) = $buf[$pos..].split_at_mut(le_bytes.len());
        target.copy_from_slice(&le_bytes);
        $pos += le_bytes.len();
    }};
}
pub(crate) use write_int;

#[macro_export]
macro_rules! write_buf {
    ($pos:expr, $buf:expr, $val:expr) => {{
        let (target, _): (&mut [_], _) = $buf[$pos..].split_at_mut($val.len());
        target.copy_from_slice($val);
        $pos += $val.len();
    }};
}
pub(crate) use write_buf;
