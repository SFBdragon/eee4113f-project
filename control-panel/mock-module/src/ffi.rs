// src/ffi.rs
//
// Declarations of the C symbols that protocol.c will define and that
// we call from Rust, plus the extern "C" symbols we export so that
// protocol.c can call back into our Rust simulation.
//
// Nothing in here is `unsafe`-masked — callers decide that.

use std::ffi::c_void;

// ---------- Types mirrored from netio.h ---------- //

pub type Status = u16;
pub type BufLen = u16;

pub const STATUS_SUCCESS: Status = 0;

pub const MAX_LORA_RECV_PACKET_LEN: usize = 64;
pub const MAX_LORA_SEND_PACKET_LEN: usize = 64;
pub const MAX_WIFI_RECV_PACKET_LEN: usize = 255;
pub const MAX_WIFI_SEND_PACKET_LEN: usize = 255;

// ---------- Types mirrored from control.h ---------- //

pub const STORAGE_BLOCK_SIZE: usize = 512;

#[repr(C)]
pub struct BlockHeader {
    pub len: u16,
    pub crc: u16,
}

#[repr(C)]
pub struct RecordHeader {
    pub len: u16,
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Policy {
    Overwrite = 0,
    Preserve = 1,
}

// ---------- Functions implemented in protocol.c, called by Rust ---------- //
//
// Add any entry points your protocol.c exposes here as you need them.
// At minimum we need the init entry point that Shaun's code provides.
// Adjust the name to match whatever protocol.c actually exports.
unsafe extern "C" {
    /// Called once at startup to hand control to Shaun's networking stack.
    /// Replace with the actual symbol name exported by protocol.c.
    pub fn protocol_init();
}
