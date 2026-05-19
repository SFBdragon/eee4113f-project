use crate::{CrcFn, LoRaAddr, wifi::common::FLAG_PING};

pub const PING_BYTES: usize = 8;

pub unsafe fn write_ping(addr: LoRaAddr, crc_fn: CrcFn, buf: *mut u8) {
    let [addr_lo, addr_hi] = addr.to_le_bytes();

    unsafe {
        buf.add(0).write(FLAG_PING);
        buf.add(1).write(0);
        buf.add(2).write(size_of::<LoRaAddr>() as u8);
        buf.add(3).write(0); // high bit of length
        buf.add(4).write(addr_lo);
        buf.add(5).write(addr_hi);

        let crc = crc_fn(buf, PING_BYTES - size_of::<u16>());
        let [crc_lo, crc_hi] = crc.to_le_bytes();

        buf.add(6).write(crc_lo);
        buf.add(7).write(crc_hi);
    }
}
