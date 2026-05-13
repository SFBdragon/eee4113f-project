// type CrcType = u16;
// const CRC_ALGORITHM: crc::Algorithm<CrcType> = crc::Algorithm {
//     width: size_of::<CrcType>() as _,
//     poly: 0xA7D3, // D3E9 in Koopman
//     init: 0xFFFF,
//     refin: false,
//     refout: false,
//     xorout: 0x0000,
//     check: 0x0000,
//     residue: 0x0000,
// };

// const CRC: crc::Crc<u16, crc::Table<1>> = crc::Crc::<u16, _>::new(&CRC_ALGORITHM);

// pub fn append_crc(buffer: &mut Vec<u8>) {
//     let mut digest = CRC.digest();
//     digest.update(&buffer);
//     let crc = digest.finalize().to_le_bytes();
//     buffer.extend_from_slice(&crc);
// }

use crc_fast::{CrcParams, checksum_with_params};

pub fn compute_crc(buf: &[u8]) -> u16 {
    let params = CrcParams::new(
        "CRC-16/CUSTOM",
        16,
        0xA7D3, // Rocksoft format, D3E9 in Koopman
        0xFFFF,
        false,
        0x0000,
        0x6F3D,
    );

    checksum_with_params(params, &buf) as _
}

pub fn compute_crc_raw(data: *const u8, len: usize) -> u16 {
    let buf = unsafe { std::slice::from_raw_parts(data, len) };
    crate::crc::compute_crc(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_crc() {
        let bytes = b"123456789";
        let crc = compute_crc(bytes);
        assert_eq!(crc, 0x6F3D);
    }
}
