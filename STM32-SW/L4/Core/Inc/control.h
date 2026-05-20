// transmission.h

#pragma once

#include <stdint.h>
#include <stdbool.h>


// ------------ Networking ------------ //

// Call `callback(ctx)` after `n` milliseconds.
// Defined by Glen. Called by Shaun for delays.
void call_after_n_ms(uint32_t n, void (*callback)());
// Cancel the active `call_after_n_ms` countdown, if there is ine.
void cancel_timeout();
// Defined by Glen.
void call_repeatedly_after_n_ms_wifi_ping(uint32_t n, void (*callback)());
// Defined by Glen.
void cancel_timeout_wifi_ping();

// Return the number of milliseconds from some arbitrary fixed epoch (e.g. since startup).
// Defined by Glen. Called by Shaun for timing information.
uint32_t get_time_since_epoch_ms();

// Configure how many seconds to be on for every N seconds.
// Units are in seconds.
// Defined by Glen. Called by Shaun to configure the LoRa receive window.
void set_lora_recv_window(uint16_t on_period, uint16_t total_period);

// CRC using poly=0xA7D3, init=0xffff, no reflect
uint16_t crc16(const uint8_t *data, uintptr_t len);

// ------------ Bulk Record/Data Storage Control ------------ //

// Defined by Glen. Called by Shaun.

// The storage page header.
typedef struct {
    uint16_t len;
    uint16_t crc;
    // [len bytes follow]
} BlockHeader;

// The size of the data units used by the rest of the API.
#define STORAGE_BLOCK_SIZE 1024

// The total number of pages available for record storage.
uint64_t storage_total_blocks();

uint64_t storage_first_readable_block();

uint64_t storage_first_protected_block();

uint64_t storage_last_readable_block();

// Read a page between `storage_first_readable_page` and `storage_last_readable_page`.
// Read the page into `buffer`. `buffer` is `STORAGE_PAGE_SIZE` large.
//
// Returns the length read in.
uint32_t read_block(uint64_t block_id, uint8_t* buffer);

typedef enum {
    // If bulk storage runs out, overwrite oldest records with new records.
    STORAGE_POLICY_OVERWRITE,
    // If bulk storage runs out, discard new records.
    STORAGE_POLICY_PRESERVE,
} Policy;

// The end-user has chosen an overwriting policy to take effect from now on.
void set_overwrite_policy(Policy policy);

// The end-user has decided that the records in pages from
// `storage_first_readable_block` (inclusive) to `upto_page` (exclusive)
// (upto_page is less than or equal to `storage_last_readable_block`)
// no longer need to be preseved on module storage.
// This must not immediately wipe any data.
// This should have an effect regardless of the current overwrite policy.
//
// This broadly indicates that the user has a copy of this data or no longer
// needs the data and it can be overwritten unless PRESERVE is set.
//
// This must adjust `storage_first_protected_block` but not
// `storage_first_readable_block` or `storage_last_readable_block`.
void allow_overwrite(uint64_t upto_block);

// Defined by Glen, called by Shaun before a WiFi transmission.
void flush_block_buffer_to_disk();


// TODO GPS STUFF ?
