use crate::{CrcFn, DeviceId, first, last, write_buf, write_int};


pub mod sender;

pub const MAX_FRAME: usize = control_sys::MAX_LORA_RECV_PACKET_LEN;
/// Two device IDs and a 16b CRC.
pub const FRAME_OVERHEAD: usize = 2 * size_of::<DeviceId>() + size_of::<u16>();
pub const MAX_PAYLOAD: usize = MAX_FRAME - FRAME_OVERHEAD;

pub const FLAG_SEQ: DeviceId = 1 << DeviceId::BITS - 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoRaFrame<'a> {
    pub controller_id_with_seq: DeviceId,
    pub module_id: DeviceId,
    pub payload: &'a [u8],
}

impl<'a> LoRaFrame<'a> {
    #[inline]
    pub fn controller_id(&self) -> DeviceId {
        self.controller_id_with_seq & !FLAG_SEQ
    }

    #[inline]
    pub fn sequence_flag(&self) -> bool {
        self.controller_id_with_seq & FLAG_SEQ != 0
    }

    pub fn parse(raw: &'a [u8], crc_fn: CrcFn) -> Result<Self, ParseError> {
        debug_assert!(raw.len() < MAX_FRAME);

        if raw.len() < FRAME_OVERHEAD { return Err(ParseError::TooShortToParse); }

        let (rem, controller_id_with_seq) = first!(DeviceId, raw);
        let (payload, module_id) = first!(DeviceId, rem);
        let (body, crc) = last!(u16, raw);

        if crc_fn(body.as_ptr(), body.len()) != crc { return Err(ParseError::BadCrc); }

        Ok(LoRaFrame { controller_id_with_seq, module_id, payload })
    }

    pub fn serialize(&self, buf: &mut [u8; MAX_FRAME], crc_fn: CrcFn) -> usize {
        let mut pos = 0;
        write_int!(pos, buf, self.controller_id_with_seq);
        write_int!(pos, buf, self.module_id);
        write_buf!(pos, buf, self.payload);
        let crc = crc_fn(buf.as_ptr(), pos);
        write_int!(pos, buf, crc);
        pos
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    TooShortToParse,
    BadCrc,
}