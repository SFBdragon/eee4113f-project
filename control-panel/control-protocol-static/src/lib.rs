#![no_std]
use panic_halt as _;

use control_protocol::{CrcFn, LoRaAddr, wifi};

// -------------------- WiFi Ping --------------------- //

#[no_mangle]
pub unsafe extern "C" fn wifi_ping_len() -> u16 {
    wifi::ping::PING_BYTES as u16
}

#[no_mangle]
pub unsafe extern "C" fn wifi_ping_write(addr: LoRaAddr, crc_fn: CrcFn, buf: *mut u8) {
    unsafe {
        wifi::ping::write_ping(addr, crc_fn, buf);
    }
}

// ------------------- WiFi Sender -------------------- //

static mut WIFI_SENDER: wifi::Sender = wifi::Sender::new();

// Call `send_next` after to send the SYN.
#[no_mangle]
pub unsafe extern "C" fn wifi_connect(
    controller_addr: LoRaAddr,
    set_timer: fn(u32),
    cancel_timer: fn(),
    get_time: fn() -> u32,
    crc_fn: CrcFn,
) {
    unsafe {
        WIFI_SENDER.connect(controller_addr, set_timer, cancel_timer, get_time, crc_fn);
    }
}

#[no_mangle]
pub unsafe extern "C" fn wifi_send_next(
    mac: *mut u64,
    data: *mut *const u8,
    len: *mut u16,
) -> bool {
    unsafe {
        WIFI_SENDER.send_next(
            mac.as_mut().unwrap().cast(),
            data.as_mut().unwrap(),
            len.as_mut().unwrap(),
        )
    }
}

#[no_mangle]
pub unsafe extern "C" fn wifi_on_recv_ack(mac: u64, data: *const u8, len: u16) -> bool {
    unsafe { WIFI_SENDER.on_recv_ack(mac.into(), core::slice::from_raw_parts(data, len as _)) }
}

#[no_mangle]
pub unsafe extern "C" fn wifi_on_timeout() -> bool {
    unsafe { WIFI_SENDER.on_timeout() }
}
