// src/injector.rs
//
// Simulates the data-acquisition side of the STM32:
//   • Every BLOCK_INTERVAL_MS a synthetic sensor record is written into
//     the storage ring via append_block().
//   • Every PRINT_INTERVAL_MS a status line is printed to stdout.
//
// This runs in its own thread and never holds the sim lock for long.

use crate::state::sim;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const BLOCK_INTERVAL_MS: u64 = 2_000; // new block every 2 s
const PRINT_INTERVAL_MS: u64 = 5_000; // status every 5 s

pub fn run_injector() {
    let mut last_block = std::time::Instant::now();
    let mut last_print = std::time::Instant::now();
    let mut seq: u32 = 0;

    loop {
        std::thread::sleep(Duration::from_millis(100));

        let now = std::time::Instant::now();

        if now.duration_since(last_block) >= Duration::from_millis(BLOCK_INTERVAL_MS) {
            last_block = now;
            let ts = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs() as u32;

            // Build a synthetic record that looks like: [RecordHeader][payload]
            // RecordHeader: u16 len, u32 unix_timestamp (little-endian, matches C struct)
            let payload = format!("synthetic-sensor-record-{seq:06}");
            let payload_bytes = payload.as_bytes();
            let len_field = payload_bytes.len() as u16;

            let mut block_data = Vec::with_capacity(6 + payload_bytes.len());
            block_data.extend_from_slice(&len_field.to_le_bytes());
            block_data.extend_from_slice(&ts.to_le_bytes());
            block_data.extend_from_slice(payload_bytes);

            let mut state = sim().lock().unwrap();
            match state.append_block(&block_data) {
                Some(id) => eprintln!("[injector] wrote block {id} (seq={seq}, ts={ts})"),
                None => eprintln!("[injector] storage full, block dropped (seq={seq})"),
            }
            seq += 1;
        }

        if now.duration_since(last_print) >= Duration::from_millis(PRINT_INTERVAL_MS) {
            last_print = now;
            let state = sim().lock().unwrap();
            println!(
                "[status] blocks: readable=[{}, {}] protected_from={}  policy={:?}  wifi={}  lora_window={}/{}s",
                state.first_readable,
                state.last_readable(),
                state.first_protected,
                if state.overwrite_policy == crate::ffi::Policy::Overwrite {
                    "Overwrite"
                } else {
                    "Preserve"
                },
                if state.wifi_up { "UP" } else { "DOWN" },
                state.lora_on_period_s,
                state.lora_total_period_s,
            );
        }
    }
}
