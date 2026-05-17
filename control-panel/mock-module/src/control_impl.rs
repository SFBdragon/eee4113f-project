// src/control_impl.rs
//
// Implements every symbol that control.h declares as "Defined by Glen".
// These are the STM32L4 core / HAL shims.  The C code (protocol.c) calls
// into these via normal linking; no dynamic dispatch needed.

use std::time::Instant;

use crate::ffi::Policy;
use crate::state::{Interval, PendingTimeout, TOTAL_BLOCKS, sim};

// Networking bootstrap

#[unsafe(no_mangle)]
pub extern "C" fn initialize_networking() {
    eprintln!("[sim] initialize_networking() called");
}

// Timers

#[unsafe(no_mangle)]
pub extern "C" fn call_after_n_ms(n: u32, callback: Option<unsafe extern "C" fn()>) {
    let Some(cb) = callback else { return };
    let fires_at = Instant::now() + std::time::Duration::from_millis(n as u64);
    let mut state = sim().lock().unwrap();
    state.timeout = Some(PendingTimeout {
        fires_at,
        callback: cb,
    });
    // eprintln!("[sim] call_after_n_ms({n}) armed");
}

#[unsafe(no_mangle)]
pub extern "C" fn cancel_timeout() {
    let mut state = sim().lock().unwrap();
    if state.timeout.take().is_some() {
        // eprintln!("[sim] cancel_timeout()");
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn call_repeatedly_after_n_ms_wifi_ping(
    n: u32,
    callback: Option<unsafe extern "C" fn()>,
) {
    let Some(cb) = callback else { return };
    let mut state = sim().lock().unwrap();
    state.ping_timer = Some(Interval {
        next: Instant::now() + std::time::Duration::from_millis(n as u64),
        interval: std::time::Duration::from_millis(n as u64),
        callback: cb,
    });
    eprintln!("[sim] call_repeatedly_after_n_ms_wifi_ping({n}) armed");
}

#[unsafe(no_mangle)]
pub extern "C" fn cancel_timeout_wifi_ping() {
    let mut state = sim().lock().unwrap();
    if state.ping_timer.take().is_some() {
        eprintln!("[sim] cancel_timeout_wifi_ping()");
    }
}

// Wallclock

#[unsafe(no_mangle)]
pub extern "C" fn get_time_since_epoch_ms() -> u32 {
    let state = sim().lock().unwrap();
    let elapsed = state.epoch.elapsed().as_millis() as u32;
    elapsed
}

// LoRa receive window

#[unsafe(no_mangle)]
pub extern "C" fn set_lora_recv_window(on_period: u16, total_period: u16) {
    let mut state = sim().lock().unwrap();
    state.lora_on_period_s = on_period;
    state.lora_total_period_s = total_period;
    eprintln!("[sim] set_lora_recv_window(on={on_period}s, total={total_period}s)");
}

// Storage

#[unsafe(no_mangle)]
pub extern "C" fn storage_total_blocks() -> u64 {
    TOTAL_BLOCKS
}

#[unsafe(no_mangle)]
pub extern "C" fn storage_first_readable_block() -> u64 {
    sim().lock().unwrap().first_readable
}

#[unsafe(no_mangle)]
pub extern "C" fn storage_first_protected_block() -> u64 {
    sim().lock().unwrap().first_protected
}

#[unsafe(no_mangle)]
pub extern "C" fn storage_last_readable_block() -> u64 {
    sim().lock().unwrap().last_readable()
}

/// Read a block into `buffer` (which is STORAGE_BLOCK_SIZE bytes).
/// Returns bytes copied, 0 if block_id is out of range.
#[unsafe(no_mangle)]
pub extern "C" fn read_block(block_id: u64, buffer: *mut u8) -> u32 {
    if buffer.is_null() {
        return 0;
    }
    let state = sim().lock().unwrap();
    if block_id < state.first_readable || block_id > state.last_readable() {
        return 0;
    }
    let idx = (block_id % TOTAL_BLOCKS) as usize;
    let block = &state.blocks[idx];
    let len = block.used;
    // SAFETY: caller guarantees buffer is STORAGE_BLOCK_SIZE bytes
    unsafe {
        std::ptr::copy_nonoverlapping(block.data.as_ptr(), buffer, len);
    }
    len as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn set_overwrite_policy(policy: Policy) {
    let mut state = sim().lock().unwrap();
    state.overwrite_policy = policy;
    eprintln!(
        "[sim] set_overwrite_policy({:?})",
        if policy == Policy::Overwrite {
            "Overwrite"
        } else {
            "Preserve"
        }
    );
}

#[unsafe(no_mangle)]
pub extern "C" fn allow_overwrite(upto_block: u64) {
    let mut state = sim().lock().unwrap();
    let last = state.last_readable();
    state.first_protected = upto_block.min(last);
    eprintln!(
        "[sim] allow_overwrite(upto={upto_block}) -> first_protected={}",
        state.first_protected
    );
}

/// No-op in sim: we have no write-back buffer.
#[unsafe(no_mangle)]
pub extern "C" fn flush_block_buffer_to_disk() {
    eprintln!("[sim] flush_block_buffer_to_disk()");
}

// Slow manual version of the CRC used in the app.
#[unsafe(no_mangle)]
pub extern "C" fn crc16(data: *const u8, len: usize) -> u16 {
    let mut crc = 0xFFFF;
    for i in 0..len {
        let byte = unsafe { data.wrapping_add(i).read() } as u16;
        crc ^= byte << 8;
        for _b in 0..8 {
            if crc & 0x8000 != 0 {
                crc = crc << 1 as u16 ^ 0xA7D3;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ------------------------------------------------------------------
// Timer tick — called from the Rust event loop, NOT from C
// ------------------------------------------------------------------

/// Fire any expired timeout callback.  Must be called without holding sim().
pub fn tick_timers() {
    // Take the timeout out of state while we check/fire it, to avoid
    // holding the lock across an FFI call.

    let mut maybe_cb = None;
    let mut maybe_ping_cb = None;

    {
        let mut state = sim().lock().unwrap();

        if let Some(timer) = &mut state.ping_timer {
            if Instant::now() >= timer.next {
                timer.next += timer.interval;
                maybe_ping_cb = Some(timer.callback);
            }
        }

        match &state.timeout {
            Some(t) if Instant::now() >= t.fires_at => maybe_cb = state.timeout.take(),
            _ => {}
        }
    };
    if let Some(t) = maybe_cb {
        eprintln!("[sim] firing timer callback");
        unsafe { (t.callback)() };
    }
    if let Some(t) = maybe_ping_cb {
        eprintln!("[sim] firing ping callback");
        unsafe { (t)() };
    }
}
