// src/netio_impl.rs
//
// Implements every symbol from netio.h that Tamryn would normally provide
// (the link/physical layer), plus hooks for Shaun's recv callbacks.
//
// Transport:
//   LoRa  → /tmp/sim_lora.sock   (SOCK_SEQPACKET, one datagram per LoRa frame)
//   WiFi  → /tmp/sim_wifi.sock   (SOCK_SEQPACKET, one datagram per WiFi frame)
//
// Wire framing for WiFi (which carries a MAC address):
//   [8 bytes macdst/macsrc big-endian][N bytes payload]
//
// LoRa has no addressing at this layer, so datagrams are raw payload.
//
// The socket paths are configurable via CLI args; see main.rs.
// These globals are written once at startup before any C code runs.

use std::net::UdpSocket;
use std::sync::{Mutex, OnceLock};
use std::time::Duration;

use rand::RngExt;

use crate::ffi::{
    BufLen, MAX_LORA_RECV_PACKET_LEN, MAX_LORA_SEND_PACKET_LEN, MAX_WIFI_RECV_PACKET_LEN,
    MAX_WIFI_SEND_PACKET_LEN, STATUS_SUCCESS, Status,
};

const MODULE_MAC_ADDRESS: u64 = 0x111111111111;

// ------------------------------------------------------------------
// Socket handles (set once from main before handing off to C)
// ------------------------------------------------------------------

pub struct Sockets {
    pub lora: UdpSocket,
    pub wifi: UdpSocket,
}

static SOCKETS: OnceLock<Mutex<Sockets>> = OnceLock::new();

pub fn init_sockets(lora: UdpSocket, wifi: UdpSocket) {
    SOCKETS.set(Mutex::new(Sockets { lora, wifi })).ok();
}

fn with_sockets<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&Sockets) -> R,
{
    SOCKETS.get().map(|m| f(&m.lock().unwrap()))
}

// ------------------------------------------------------------------
// LoRa
// ------------------------------------------------------------------

#[unsafe(no_mangle)]
pub extern "C" fn initialize_lora() -> Status {
    eprintln!("[sim] initialize_lora()");
    STATUS_SUCCESS
}

/// Send `len` bytes from `data` out the LoRa socket.
#[unsafe(no_mangle)]
pub extern "C" fn send_lora_packet(data: *const u8, len: BufLen) -> Status {
    if data.is_null() || len as usize > MAX_LORA_SEND_PACKET_LEN {
        eprintln!("[sim] send_lora_packet: bad args (len={len})");
        return 1;
    }
    let slice = unsafe { std::slice::from_raw_parts(data, len as usize) };
    eprintln!(
        "[sim] send_lora_packet({len} bytes): {:02x?}",
        &slice[..slice.len().min(16)]
    );

    if crate::LORA_RELIABILITY_TEST {
        if rand::random::<f64>() < crate::LORA_SEND_DROP_RATE {
            return STATUS_SUCCESS;
        }
    }

    let mut buf = slice.to_vec();
    if crate::LORA_CORRUPT_TEST {
        flip_bits_with_probability(&mut buf, crate::LORA_SEND_BITFLIP_RATE);
    };

    match with_sockets(|s| s.lora.send_to(&buf, "127.0.0.1:12000")) {
        Some(Ok(_)) => STATUS_SUCCESS,
        Some(Err(e)) => {
            eprintln!("[sim] send_lora_packet error: {e}");
            3
        }
        None => {
            eprintln!("[sim] send_lora_packet: sockets not initialized");
            5
        }
    }
}

// ------------------------------------------------------------------
// WiFi
// ------------------------------------------------------------------

#[unsafe(no_mangle)]
pub extern "C" fn initialize_wifi() -> Status {
    eprintln!("[sim] initialize_wifi()");
    STATUS_SUCCESS
}

#[unsafe(no_mangle)]
pub extern "C" fn power_up_wifi() -> Status {
    crate::state::sim().lock().unwrap().wifi_up = true;
    eprintln!("[sim] power_up_wifi()");
    STATUS_SUCCESS
}

#[unsafe(no_mangle)]
pub extern "C" fn power_down_wifi() -> Status {
    crate::state::sim().lock().unwrap().wifi_up = false;
    eprintln!("[sim] power_down_wifi()");
    STATUS_SUCCESS
}

/// Wire format: [8 bytes macdst BE][payload]
#[unsafe(no_mangle)]
pub extern "C" fn send_wifi_packet(
    macdst: u64,
    data: *const u8,
    len: u16,
) -> std::os::raw::c_short {
    if data.is_null() || len as usize > MAX_WIFI_SEND_PACKET_LEN {
        eprintln!("[sim] send_wifi_packet: bad args");
        return -1;
    }
    let payload = unsafe { std::slice::from_raw_parts(data, len as usize) };
    let mut frame = Vec::with_capacity(16 + payload.len());
    frame.extend_from_slice(&MODULE_MAC_ADDRESS.to_le_bytes());
    frame.extend_from_slice(&macdst.to_le_bytes());
    frame.extend_from_slice(payload);
    eprintln!(
        "[sim] send_wifi_packet(src={MODULE_MAC_ADDRESS:#018x} dst={macdst:#018x}, {len} bytes): {:02x?}",
        &payload[..payload.len().min(24)]
    );

    std::thread::sleep(Duration::from_millis(10));

    if crate::WIFI_RELIABILITY_TEST {
        if rand::random::<f64>() < crate::WIFI_SEND_DROP_RATE {
            return 0;
        }
    }

    if crate::WIFI_CORRUPT_TEST {
        flip_bits_with_probability(&mut frame, crate::WIFI_SEND_BITFLIP_RATE);
    }

    match with_sockets(|s| s.wifi.send_to(&frame, "127.0.0.1:12010")) {
        Some(Ok(_)) => 0,
        Some(Err(e)) => {
            eprintln!("[sim] send_wifi_packet error: {e}");
            -1
        }
        None => {
            eprintln!("[sim] send_wifi_packet: sockets not initialized");
            -1
        }
    }
}

// ------------------------------------------------------------------
// Inbound packet delivery — called from our Rust socket reader threads.
//
// These call back into Shaun's protocol.c via the recv_*_packet symbols
// he defines.  We declare the C symbols as extern here.
// ------------------------------------------------------------------

unsafe extern "C" {
    /// Defined by Shaun (protocol.c).
    fn recv_lora_packet(data: *const u8, len: BufLen);
    /// Defined by Shaun (protocol.c).
    fn recv_wifi_packet(macsrc: *const u64, data: *const u8, len: u16);
}

/// Read one LoRa datagram from the socket and deliver it to Shaun's code.
/// Returns false if the socket had no data or errored.
pub fn poll_lora_recv() -> bool {
    // eprintln!("[sim] recv_lora_packet poll");

    let mut buf = [0u8; MAX_LORA_RECV_PACKET_LEN];
    let result = with_sockets(|s| s.lora.recv(&mut buf));
    match result {
        Some(Ok(n)) if n > 0 => {
            eprintln!(
                "[sim] recv_lora_packet({n} bytes): {:02x?}",
                &buf[..n.min(16)]
            );

            if crate::LORA_RELIABILITY_TEST {
                if rand::random::<f64>() < crate::LORA_RECV_DROP_RATE {
                    return false;
                }
            }

            if crate::LORA_CORRUPT_TEST {
                flip_bits_with_probability(&mut buf, crate::LORA_RECV_BITFLIP_RATE);
            }

            // SAFETY: buf is valid, n <= MAX_LORA_RECV_PACKET_LEN
            unsafe { recv_lora_packet(buf.as_ptr(), n as BufLen) };
            true
        }
        Some(Err(e)) if e.kind() == std::io::ErrorKind::WouldBlock => false,
        Some(Err(e)) => {
            eprintln!("[sim] lora recv error: {e}");
            false
        }
        _ => false,
    }
}

/// Read one WiFi datagram from the socket and deliver it to Shaun's code.
/// Wire format: [8 bytes macsrc BE][payload]
pub fn poll_wifi_recv() -> bool {
    let mut buf = [0u8; 8 + MAX_WIFI_RECV_PACKET_LEN];
    let result = with_sockets(|s| s.wifi.recv(&mut buf));
    match result {
        Some(Ok(n)) if n > 8 => {
            let macsrc = u64::from_le_bytes(buf[..8].try_into().unwrap());
            let macdst = u64::from_le_bytes(buf[8..16].try_into().unwrap());
            let payload_len = n - 16;
            if macdst != 0xffffffffffff && macdst != MODULE_MAC_ADDRESS {
                eprintln!(
                    "[sim] recv_wifi_packet(src={macsrc:#018x}, payload={:02x?}) BAD DESTINATION",
                    &buf[16..n]
                );
                return false;
            }
            eprintln!(
                "[sim] recv_wifi_packet(src={macsrc:#018x}, payload={:02x?})",
                &buf[16..n]
            );

            if crate::WIFI_RELIABILITY_TEST {
                if rand::random::<f64>() < crate::WIFI_RECV_DROP_RATE {
                    return false;
                }
            }

            if crate::WIFI_CORRUPT_TEST {
                flip_bits_with_probability(&mut buf, crate::WIFI_RECV_BITFLIP_RATE);
            }

            // SAFETY: pointers valid, sizes bounded
            unsafe {
                recv_wifi_packet(
                    &macsrc as *const u64,
                    buf[16..].as_ptr(),
                    payload_len as u16,
                )
            };
            true
        }
        Some(Err(e)) if e.kind() == std::io::ErrorKind::WouldBlock => false,
        Some(Err(e)) => {
            eprintln!("[sim] wifi recv error: {e}");
            false
        }
        _ => {
            eprint!("wifi recv of some description");

            false
        }
    }
}

/// Flips bits in a mutable byte slice based on a probability `p`.
///
/// Slow implementation but should be good enough.
fn flip_bits_with_probability(data: &mut [u8], p: f64) {
    let mut rng = rand::rng();

    for byte in data.iter_mut() {
        for bit_idx in 0..8 {
            if rng.random_bool(p) {
                *byte ^= 1 << bit_idx;
            }
        }
    }
}
