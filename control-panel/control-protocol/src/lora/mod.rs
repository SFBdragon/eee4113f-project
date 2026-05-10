use crate::{CrcFn, LoRaAddr, first, last, write_buf, write_int};

pub const MAX_FRAME: usize = crate::phy::MAX_LORA_RECV_PACKET_LEN;
/// Two device IDs and a 16b CRC.
pub const FRAME_OVERHEAD: usize = 2 * size_of::<LoRaAddr>() + size_of::<u16>();
pub const MAX_PAYLOAD: usize = MAX_FRAME - FRAME_OVERHEAD;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoRaFrame<'a> {
    pub con_addr: LoRaAddr,
    pub mod_addr: LoRaAddr,
    pub sequence_flag: bool,
    pub payload: &'a [u8],
}

impl<'a> LoRaFrame<'a> {
    const SEQUENCE_BIT: u16 = LoRaAddr::RESERVED_BIT;

    pub fn sequence_bit(&self) -> u16 {
        if self.sequence_flag {
            Self::SEQUENCE_BIT
        } else {
            0
        }
    }

    pub fn first_u16(controller_addr: LoRaAddr, sequence_flag: bool) -> u16 {
        controller_addr.to_raw() | if sequence_flag { Self::SEQUENCE_BIT } else { 0 }
    }

    pub fn parse(raw: &'a [u8], crc_fn: CrcFn) -> Result<Self, ParseError> {
        debug_assert!(raw.len() <= MAX_FRAME);

        if raw.len() < FRAME_OVERHEAD {
            return Err(ParseError::TooShortToParse);
        }

        let (rem, controller_addr_with_seq) = first!(u16, raw);
        let (rem, module_addr) = first!(u16, rem);
        let (body, crc) = last!(u16, raw);

        if crc_fn(body.as_ptr(), body.len()) != crc {
            return Err(ParseError::BadCrc);
        }

        Ok(LoRaFrame {
            con_addr: LoRaAddr::from_raw(controller_addr_with_seq & !Self::SEQUENCE_BIT),
            mod_addr: LoRaAddr::from_raw(module_addr),
            sequence_flag: controller_addr_with_seq & Self::SEQUENCE_BIT != 0,
            payload: last!(u16, rem).0,
        })
    }

    pub fn serialize(&self, buf: &mut [u8; MAX_FRAME], crc_fn: CrcFn) -> usize {
        let mut pos = 0;
        write_int!(pos, buf, self.con_addr.to_raw() | self.sequence_bit());
        write_int!(pos, buf, self.mod_addr);
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
