//! common.rs — shared frame definitions, constants, seq arithmetic
//! no_std, no alloc

use crate::{
    CrcFn,
    byteutils::{first, last, write_buf, write_int},
    call_crc_fn,
};

// ── Constants ────────────────────────────────────────────────────────────────

/// Maximum payload size we can give to the link-layer.
pub const MAX_DATAGRAM: usize = crate::phy::MAX_WIFI_SEND_PACKET_LEN;
/// CRC length.
pub const CRC_LEN: usize = 2;
/// Number of bytes of overhead per non-ACK datagram.
pub const DG_OVERHEAD: usize = 1 + size_of::<SeqNumType>() + 2 + CRC_LEN; // flags, seq, len, crc
/// The maximum application-layer payload size.
pub const MAX_PAYLOAD: usize = MAX_DATAGRAM - DG_OVERHEAD;

pub type SeqNumType = u8;
pub const SEQ_SPACE: usize = 1 << SeqNumType::BITS;
/// Selective-repeat constraint: SEQ_SPACE >= 2 * MAX_WINDOW.
pub const WINDOW_LEN: usize = 8;
pub type WindowBitMap = u8;

pub const PING_PERIOD_MS: usize = 1000;
pub const PING_MISSED_THRESH: usize = 3;

// ── Flags ────────────────────────────────────────────────────────────────────

// A connectionless ping. This is used to test the reachability of WiFi.
// This should not affect any connection state.
pub const FLAG_PING: u8 = 0b0000_0001;

/// Sync the receiver's sequence tracking.
/// This creates a new connection effectively.
/// Because the current implementation only supports one active connection,
/// this will cause the receiver to drop the previous connection state, so
/// finish up the previous transmission first (close out with FIN).
pub const FLAG_SYN: u8 = 0b0000_0010;
/// The end flag. The end of a message. Facilitates fragmentation and reassembly.
///
/// Each message implicitly starts after the previous END.
/// An END on consecutive messages means a single-frame message was sent.
pub const FLAG_END: u8 = 0b0000_1000;

// Note that the above can be combined. A small one-message transmission may SYN,BGN,END,FIN.
// Which results in effectively connectionless communication. Useful for pings.

/// This packet is an acknowledgement from the receiver to the sender about
/// which packets were received successfully.
pub const FLAG_ACK: u8 = 0b0100_0000;
/// RST packets are a flavour of ACK packet that tells the sender to
/// reconnect or go away.
/// It indicates that the sender does not have a connection with the receiver.
pub const FLAG_RST: u8 = 0b1000_0000;

// ── Sequence arithmetic (mod 256) ─────────────────────────────────────────────

/// Add to a sequence number in a module fashion.
#[inline]
pub fn seq_add(a: u8, b: u8) -> u8 {
    a.wrapping_add(b)
}

/// Returns true if `a` is strictly ahead of `b` within a half-space of SEQ_SPACE.
/// Standard sliding-window "is seq in window" comparator.
#[inline]
pub fn seq_gt(a: u8, b: u8) -> bool {
    // a > b in circular sense iff (a - b) mod 256 is in (0, 128)
    let diff = a.wrapping_sub(b);
    diff > 0 && diff < 128
}

/// Check if a sequence number is within [base, base+window)
/// in a module fashion.
#[inline]
pub fn seq_in_window(seq: u8, base: u8, window: u8) -> bool {
    let offset = seq.wrapping_sub(base);
    offset < window
}

// ── Frame structures ──────────────────────────────────────────────────────────

/// Parsed DATA frame. Borrows payload from the raw buffer — no copy.
#[derive(Debug, PartialEq, Eq)]
pub struct DataFrame<'a> {
    pub flags: u8,
    pub seq: u8,
    pub len: u16,
    pub payload: &'a [u8],
}

impl<'a> DataFrame<'a> {
    pub const SEQ_OFFSET: usize = 1;

    /// Serialise into `buf`. Returns the number of bytes written, or None if
    /// buf is too small. CRC is appended by this function using `crc_fn`.
    pub fn serialize(&self, buf: &mut [u8; MAX_DATAGRAM], crc_fn: CrcFn) -> usize {
        debug_assert!(self.payload.len() <= MAX_PAYLOAD);

        let mut pos = 0;
        write_int!(pos, buf, self.flags & !FLAG_ACK);
        write_int!(pos, buf, self.seq);
        write_int!(pos, buf, self.len);
        write_buf!(pos, buf, self.payload);

        let crc = call_crc_fn(crc_fn, &buf[..pos]);
        write_int!(pos, buf, crc);

        pos
    }

    /// Parse from raw bytes. Validates CRC. Returns None on malformed input.
    pub fn parse(buf: &'a [u8], crc_fn: CrcFn) -> Result<Self, ParseError> {
        if buf.len() < 3 + CRC_LEN {
            return Err(ParseError::TooShortToParse);
        }

        let (rem, flags) = first!(u8, buf);
        if flags & (FLAG_ACK | FLAG_RST) != 0 {
            return Err(ParseError::WrongType);
        }

        let (rem, seq) = first!(u8, rem);
        let (rem, len) = first!(u16, rem);

        let expected_len = len as usize + DG_OVERHEAD;
        if buf.len() != expected_len {
            return Err(ParseError::BadLength);
        }

        let (body, crc) = last!(u16, buf);

        // Validate CRC over header + payload
        if call_crc_fn(crc_fn, body) != crc {
            return Err(ParseError::BadCrc);
        }

        Ok(DataFrame {
            flags,
            seq,
            len,
            payload: last!(u16, rem).0,
        })
    }
}

/// Parsed ACK frame.
#[derive(Debug, PartialEq, Eq)]
pub struct AckFrame {
    pub flags: u8,
    /// The receiver's current recv_base. The sender uses this to align the
    /// bitmap: bit i corresponds to sequence number (ack_base + i) mod 256.
    pub ack_base: u8,

    /// Set bits indicate successfully received packets.
    /// Unset bits indicate no reception of the packet.
    ///
    /// Each bit index corresponds to the packets' offesets from recv_base.
    pub bitmap: WindowBitMap,
}

/// The length of an ACK datagram.
pub const ACK_SIZE: usize = 1 + 1 + size_of::<WindowBitMap>() + CRC_LEN;

impl AckFrame {
    /// Create a sync (SYN) ack frame.
    /// This tells the sender that we are prepared to connect with them,
    /// and they can start sending us data from `seq` as specified in the SYN frame.
    pub fn syn(seq: u8) -> Self {
        AckFrame {
            flags: FLAG_ACK | FLAG_SYN,
            ack_base: seq,
            bitmap: 0,
        }
    }

    /// Create a reset (RST) ack frame.
    /// This tells the sender that we don't have a connection with them and
    /// they need to connect (send a SYN) before we accept data from them.
    pub fn rst() -> Self {
        AckFrame {
            flags: FLAG_ACK | FLAG_RST,
            ack_base: 0,
            bitmap: 0,
        }
    }

    /// Create a normal ack frame.
    /// This tells the sender what data we have received.
    pub fn ack(ack_base: u8, bitmap: WindowBitMap) -> Self {
        AckFrame {
            flags: FLAG_ACK,
            ack_base,
            bitmap,
        }
    }

    /// Writes `ACK_SIZE` bytes to `buf`.
    pub fn to_buf(&self, buf: &mut [u8; ACK_SIZE], crc_fn: CrcFn) -> Option<usize> {
        let mut pos = 0;
        write_int!(pos, buf, FLAG_ACK | self.flags);
        write_int!(pos, buf, self.ack_base);
        write_int!(pos, buf, self.bitmap);

        let crc = call_crc_fn(crc_fn, &buf[..pos]);
        write_int!(pos, buf, crc);

        Some(pos)
    }

    /// Returns the ACK bytes.
    pub fn to_bytes(&self, crc_fn: CrcFn) -> [u8; ACK_SIZE] {
        let mut buf = [0u8; ACK_SIZE];
        self.to_buf(&mut buf, crc_fn).unwrap();
        buf
    }

    pub fn parse(buf: &[u8], crc_fn: CrcFn) -> Result<Self, ParseError> {
        if buf.len() < ACK_SIZE {
            return Err(ParseError::TooShortToParse);
        }

        let (rem, flags) = first!(u8, buf);
        if flags & FLAG_ACK == 0 {
            return Err(ParseError::WrongType);
        }

        let (rem, ack_base) = first!(u8, rem);
        let (_, bitmap) = first!(WindowBitMap, rem);
        let (rx_body, rx_crc) = last!(u16, buf);

        if call_crc_fn(crc_fn, rx_body) != rx_crc {
            return Err(ParseError::BadCrc);
        }

        Ok(AckFrame {
            flags,
            ack_base,
            bitmap,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    TooShortToParse,
    BadLength,
    WrongType,
    BadCrc,
}

#[cfg(any(test, feature = "test-utils"))]
pub mod test_utils {
    use crate::wifi::sender::test_utils::crc16;

    use super::*;

    // /// Fake CRC: XOR all bytes into the low byte, high byte always 0xAB.
    // /// Deterministic and collision-free enough for unit tests.
    // pub extern "C" fn test_crc(data: *const u8, len: usize) -> u16 {
    //     let buf = unsafe { std::slice::from_raw_parts(data, len) };
    //     let low = buf.iter().fold(0u8, |acc, &b| acc ^ b);
    //     0xAB00 | low as u16
    // }

    /// Serialize a DATA frame manually, bypassing the Sender state machine.
    pub fn make_data_frame(seq: u8, flags: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; MAX_DATAGRAM];
        let frame = DataFrame {
            flags,
            seq,
            len: payload.len() as _,
            payload,
        };
        let len = frame.serialize(&mut buf, crc16);
        buf[..len].to_vec()
    }

    pub fn make_ack(bitmap: WindowBitMap) -> AckFrame {
        make_ack_with_base(0, bitmap)
    }

    pub fn make_ack_with_base(ack_base: u8, bitmap: WindowBitMap) -> AckFrame {
        AckFrame {
            flags: FLAG_ACK,
            ack_base,
            bitmap,
        }
    }

    pub extern "C" fn noop_timer_set(_ms: u32) {}
    pub extern "C" fn noop_timer_cancel() {}
    pub extern "C" fn time_zero() -> u32 {
        0
    }
}

#[cfg(test)]
pub mod tests {

    use crate::wifi::sender::test_utils::crc16;

    use super::test_utils::*;
    use super::*;

    // Test frame serialisation/deserialisation, CRC gating, seq arithmetic.

    // =========================================================================
    // common — seq arithmetic
    // =========================================================================

    #[test]
    fn seq_add_wraps() {
        assert_eq!(seq_add(255, 1), 0);
        assert_eq!(seq_add(200, 60), 4);
        assert_eq!(seq_add(0, 0), 0);
    }

    #[test]
    fn seq_gt_basic() {
        assert!(seq_gt(1, 0));
        assert!(!seq_gt(0, 1));
        assert!(!seq_gt(5, 5));
    }

    #[test]
    fn seq_gt_wraparound() {
        // 0 is "after" 255 in circular sense (diff = 1, in (0,128))
        assert!(seq_gt(0, 255));
        // 127 is after 0
        assert!(seq_gt(127, 0));
        // 128 is NOT after 0 (diff == 128, boundary is exclusive)
        assert!(!seq_gt(128, 0));
    }

    #[test]
    fn seq_in_window_basic() {
        // Window [10, 10+64)
        assert!(seq_in_window(10, 10, 64));
        assert!(seq_in_window(73, 10, 64));
        assert!(!seq_in_window(74, 10, 64));
        assert!(!seq_in_window(9, 10, 64));
    }

    #[test]
    fn seq_in_window_wraparound() {
        // Window base = 250, window = 64: covers 250..=255 and 0..=57
        assert!(seq_in_window(255, 250, 64));
        assert!(seq_in_window(0, 250, 64));
        assert!(seq_in_window(57, 250, 64));
        assert!(!seq_in_window(58, 250, 64));
        assert!(!seq_in_window(249, 250, 64));
    }

    // =========================================================================
    // common — DataFrame serialise / parse round-trip
    // =========================================================================

    #[test]
    fn data_frame_roundtrip_normal() {
        let payload = b"hello world";
        let raw = make_data_frame(42, 0, payload);
        let parsed = DataFrame::parse(&raw, crc16).unwrap();
        assert_eq!(parsed.seq, 42);
        assert_eq!(parsed.flags & FLAG_ACK, 0);
        assert_eq!(parsed.payload, payload);
        assert_eq!(parsed.len as usize, payload.len());
    }

    #[test]
    fn data_frame_roundtrip_empty_payload() {
        let raw = make_data_frame(0, 0, b"");
        let parsed = DataFrame::parse(&raw, crc16).unwrap();
        assert_eq!(parsed.payload.len(), 0);
        assert_eq!(parsed.seq, 0);
    }

    #[test]
    fn data_frame_roundtrip_max_payload() {
        let payload = vec![0xAA; MAX_PAYLOAD];
        let raw = make_data_frame(255, 0, &payload);
        let parsed = DataFrame::parse(&raw, crc16).unwrap();
        assert_eq!(parsed.payload, payload.as_slice());
    }

    #[test]
    fn data_frame_fin_flag_roundtrip() {
        for syn in [0, FLAG_SYN] {
            for end in [0, FLAG_END] {
                let raw = make_data_frame(7, syn | end, b"fin");
                let parsed = DataFrame::parse(&raw, crc16).unwrap();
                assert_eq!(parsed.flags, syn | end);
            }
        }
    }

    #[test]
    fn data_frame_parse_rejects_ack_bit() {
        // A frame with FLAG_ACK set must be rejected by DataFrame::parse
        let mut raw = make_data_frame(0, 0, b"x");
        raw[0] |= FLAG_ACK;
        // CRC will also be wrong now, but the ACK-bit check comes first
        assert_eq!(DataFrame::parse(&raw, crc16), Err(ParseError::WrongType));
    }

    #[test]
    fn data_frame_parse_rejects_bad_crc() {
        let mut raw = make_data_frame(1, 0, b"data");
        // Corrupt the first CRC byte
        let n = raw.len();
        raw[n - 2] ^= 0xFF;
        assert_eq!(DataFrame::parse(&raw, crc16), Err(ParseError::BadCrc));
    }

    #[test]
    fn data_frame_parse_rejects_truncated() {
        let raw = make_data_frame(0, 0, b"abc");
        // Feed only the header, no payload, no CRC
        assert_eq!(
            DataFrame::parse(&raw[..3], crc16),
            Err(ParseError::TooShortToParse)
        );
    }

    #[test]
    fn data_frame_serialize_clears_ack_bit() {
        // Even if the caller passes FLAG_ACK in flags, serialize must clear it
        let raw = make_data_frame(0, FLAG_ACK, b"x");
        assert_eq!(raw[0] & FLAG_ACK, 0);
    }

    // =========================================================================
    // common — AckFrame serialise / parse round-trip
    // =========================================================================

    #[test]
    fn ack_frame_roundtrip() {
        let bitmaps = [0, WindowBitMap::MAX, 0xAA];
        let bases = [0, (SEQ_SPACE - 1) as u8, 0xAA];

        for bitmap in bitmaps {
            for base in bases {
                let ack = make_ack_with_base(base, bitmap);
                let bytes = ack.to_bytes(crc16);
                let parsed = AckFrame::parse(&bytes, crc16).unwrap();
                assert_eq!(parsed, ack);
            }
        }
    }

    #[test]
    fn ack_frame_parse_rejects_missing_ack_bit() {
        let mut bytes = make_ack(1).to_bytes(crc16);
        bytes[0] &= !FLAG_ACK; // clear the ACK bit
        // CRC will also be wrong, but the flags are checked first
        assert_eq!(AckFrame::parse(&bytes, crc16), Err(ParseError::WrongType));
    }

    #[test]
    fn ack_frame_parse_rejects_bad_crc() {
        let mut bytes = make_ack(0xFF).to_bytes(crc16);
        bytes[ACK_SIZE - 1] ^= 0xFF;
        assert_eq!(AckFrame::parse(&bytes, crc16), Err(ParseError::BadCrc));
    }

    #[test]
    fn ack_frame_parse_rejects_truncated() {
        let bytes = make_ack(0).to_bytes(crc16);
        assert_eq!(
            AckFrame::parse(&bytes[..(ACK_SIZE - 1)], crc16),
            Err(ParseError::TooShortToParse)
        );
    }
}
