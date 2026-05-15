// src/state.rs
//
// All mutable state for the simulated STM32L4 core, protected behind a
// single Mutex so the C callbacks (called from any thread) stay safe.

use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use crate::ffi::{Policy, STORAGE_BLOCK_SIZE};

// ---------- Storage ---------- //

/// One 512-byte storage block.
pub struct Block {
    pub data: [u8; STORAGE_BLOCK_SIZE],
    pub used: usize, // bytes written into data[]
}

impl Block {
    pub fn zeroed() -> Self {
        Self {
            data: [0u8; STORAGE_BLOCK_SIZE],
            used: 0,
        }
    }
}

// ---------- Timeout callback ---------- //

pub struct PendingTimeout {
    pub fires_at: Instant,
    pub callback: unsafe extern "C" fn(),
}

pub struct Interval {
    pub next: Instant,
    pub interval: Duration,
    pub callback: unsafe extern "C" fn(),
}

// SAFETY: the ctx pointer comes from C and may be anything; we are
// responsible for calling it on the right thread.
unsafe impl Send for PendingTimeout {}

// ---------- Core shared state ---------- //

pub struct SimState {
    // -- Storage --
    /// Ring buffer of blocks.  Index 0 is the absolute first block ever.
    pub blocks: Vec<Block>,
    /// Logical cursor: first block the protocol may read.
    pub first_readable: u64,
    /// First block that may NOT be overwritten (user hasn't released it yet).
    pub first_protected: u64,
    /// One-past-last writable block (wraps in ring fashion).
    pub write_head: u64,

    pub overwrite_policy: Policy,

    // -- Timing --
    pub epoch: Instant,

    // -- Pending timer --
    pub timeout: Option<PendingTimeout>,
    pub ping_timer: Option<Interval>,

    // -- LoRa window config --
    pub lora_on_period_s: u16,
    pub lora_total_period_s: u16,

    // -- WiFi power state --
    pub wifi_up: bool,
}

/// Total number of blocks in our simulated storage ring.
pub const TOTAL_BLOCKS: u64 = 256;

impl SimState {
    fn new() -> Self {
        let mut blocks = Vec::with_capacity(TOTAL_BLOCKS as usize);
        for _ in 0..TOTAL_BLOCKS {
            blocks.push(Block::zeroed());
        }
        Self {
            blocks,
            first_readable: 0,
            first_protected: 0,
            write_head: 0,
            overwrite_policy: Policy::Overwrite,
            epoch: Instant::now(),
            timeout: None,
            ping_timer: None,
            lora_on_period_s: 1,
            lora_total_period_s: 10,
            wifi_up: false,
        }
    }
}

// ---------- Global singleton ---------- //

static SIM: OnceLock<Mutex<SimState>> = OnceLock::new();

pub fn sim() -> &'static Mutex<SimState> {
    SIM.get_or_init(|| Mutex::new(SimState::new()))
}

// ---------- Storage helpers ---------- //

impl SimState {
    /// Append `data` as a new block.  Handles the overwrite / preserve policy.
    /// Returns the block id assigned, or None if storage is full and policy is Preserve.
    pub fn append_block(&mut self, data: &[u8]) -> Option<u64> {
        let total = TOTAL_BLOCKS;
        let used_blocks = self.write_head - self.first_readable;

        if used_blocks >= total {
            match self.overwrite_policy {
                Policy::Preserve => return None,
                Policy::Overwrite => {
                    // Advance readable past the oldest block to make room,
                    // but never past first_protected.
                    if self.first_readable < self.first_protected {
                        // Can't overwrite protected data; behave like Preserve.
                        return None;
                    }
                    self.first_readable += 1;
                }
            }
        }

        let idx = (self.write_head % total) as usize;
        let block = &mut self.blocks[idx];
        let copy_len = data.len().min(STORAGE_BLOCK_SIZE);
        block.data[..copy_len].copy_from_slice(&data[..copy_len]);
        block.used = copy_len;

        let id = self.write_head;
        self.write_head += 1;
        Some(id)
    }

    pub fn last_readable(&self) -> u64 {
        if self.write_head == 0 {
            0
        } else {
            self.write_head - 1
        }
    }
}
