#![feature(linkage)]
#![no_std]

// #[cfg(not(feature = "for-rust"))]
// use panic_halt as _;

// #[cfg(feature = "for-rust")]
#[linkage = "weak"]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

use control_protocol::{CrcFn, LoRaAddr, wifi};

// -------------------- WiFi Ping --------------------- //

#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_ping_len() -> u16 {
    wifi::ping::PING_BYTES as u16
}

// Write the bytes of a ping to a buffer. Assumes the buffer length is `wifi_ping_len`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_ping_write(addr: LoRaAddr, crc_fn: CrcFn, buf: *mut u8) {
    unsafe {
        wifi::ping::write_ping(addr, crc_fn, buf);
    }
}

// ------------------- WiFi Sender -------------------- //

struct StaticState(core::cell::UnsafeCell<wifi::Sender>);
unsafe impl Sync for StaticState {}

static WIFI_SENDER: StaticState = StaticState(core::cell::UnsafeCell::new(wifi::Sender::new()));

// Returns true if `wifi_send_next` should be called.
// (Call `send_next` after to send the SYN.)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_connect(
    controller_addr: LoRaAddr,
    set_timer: extern "C" fn(u32),
    cancel_timer: extern "C" fn(),
    get_time: extern "C" fn() -> u32,
    crc_fn: CrcFn,
) -> bool {
    unsafe {
        WIFI_SENDER.0.get().as_mut_unchecked().connect(
            controller_addr,
            set_timer,
            cancel_timer,
            get_time,
            crc_fn,
        );
        true
    }
}

// Returns true if `wifi_send_next` should be called.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_send_next(
    mac: *mut u64,
    data: *mut *const u8,
    len: *mut u16,
) -> bool {
    unsafe {
        WIFI_SENDER.0.get().as_mut_unchecked().send_next(
            mac.cast::<wifi::Mac>().as_mut().unwrap(),
            data.as_mut().unwrap(),
            len.as_mut().unwrap(),
        )
    }
}

// Returns true if `wifi_send_next` should be called.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_on_recv_ack(mac: u64, data: *const u8, len: u16) -> bool {
    unsafe {
        WIFI_SENDER
            .0
            .get()
            .as_mut_unchecked()
            .on_recv_ack(mac.into(), core::slice::from_raw_parts(data, len as _))
    }
}

// Returns true if `wifi_send_next` should be called.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_on_timeout() -> bool {
    unsafe { WIFI_SENDER.0.get().as_mut_unchecked().on_timeout() }
}

// Returns true if `wifi_send_next` should be called.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_push_message(data: *const u8, len: u16) -> bool {
    unsafe {
        let slice = core::slice::from_raw_parts(data, len as _);
        WIFI_SENDER.0.get().as_mut_unchecked().push_message(slice)
    }
}

// Returns how many bytes may be buffered for sending with `wifi_push_message`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wifi_available_payload_bytes() -> u32 {
    unsafe {
        WIFI_SENDER
            .0
            .get()
            .as_mut_unchecked()
            .available_payload_bytes()
    }
}
