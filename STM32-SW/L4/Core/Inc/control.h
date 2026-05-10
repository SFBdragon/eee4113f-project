// transmission.h

#pragma once

#include <stdint.h>
#include <stdbool.h>


// ------------ Networking ------------ //


// - Initializes networking subsystem.
// - Starts LoRa listening.
//
// Defined by Shaun. Called by Glen at startup.
void initialize_networking();

// Call `callback(ctx)` after `n` milliseconds.
// Defined by Glen. Called by Shaun for delays.
void call_after_n_ms(uint32_t n, void (*callback)(void *ctx), void *ctx);

// Cancel the active `call_after_n_ms` countdown, if there is ine.
void cancel_timeout();

// Return the number of milliseconds from some arbitrary fixed epoch (e.g. since startup).
// Defined by Glen. Called by Shaun for timing information.
uint32_t get_time_since_epoch_ms();

// Configure how many seconds to be on for every N seconds.
// Units are in seconds.
// Defined by Glen. Called by Shaun to configure the LoRa receive window.
void set_lora_recv_window(uint16_t on_period, uint16_t total_period);

// ------------ Bulk Record/Data Storage Control ------------ //

// Defined by Glen. Called by Shaun.

// The storage page header.
typedef struct {
    // metadata
    // metadata
    // metadata
    uint32_t blk;
    // len?
    uint16_t len;
    // crc?
    uint32_t crc;
} PageHeader;

// The size of the data units used by the rest of the API.
#define STORAGE_BLOCK_SIZE /* = <glen plz specify> */

// The total number of pages available for record storage.
uint32_t storage_total_blocks();

uint32_t storage_first_readable_block();

uint32_t storage_first_protected_block();

uint32_t storage_last_readable_block();

// This is effectively `total_blocks_written / total_blocks`.
// What I really want to know is whether blocks[n] is the first block
// to be written there, or if it's been overwritten more recently.
//
// If you're doing a ring buffer, this is equal to the number of times
// the write pointer wrapped around.
uint16_t storage_block_generation();

// Read a page between `storage_first_readable_page` and `storage_last_readable_page`.
// Read the page into `buffer`. `buffer` is `STORAGE_PAGE_SIZE` large.
//
// Returns the length read in.
uint32_t read_page(uint32_t page_index, uint8_t* buffer);

typedef enum {
    // If bulk storage runs out, overwrite oldest records with new records.
    OVERWRITE,
    // If bulk storage runs out, discard new records.
    PRESERVE,
    // Don't write any new blocks. Discard future incoming data.
    // Don't delete any existing measurements.
    READONLY,
} Policy;

// The end-user has chosen an overwriting policy to take effect from now on.
void set_overwrite_policy(Policy policy);

// The end-user has decided that the records in pages from
// `storage_first_readable_block` (inclusive) to `upto_page` (exclusive)
// (upto_page is less than or equal to `storage_last_readable_block`)
// no longer need to be preseved on telemeter storage.
// This should not need to immediately wipe any data.
// This should have an effect regardless of the current overwrite policy.
//
// This broadly indicates that the user has a copy of this data or no longer
// needs the data and it can be overwritten unless PRESERVE is set.
//
// This must adjust `storage_first_protected_block` but not
// `storage_first_readable_block` or `storage_last_readable_block`.
void allow_overwrite(uint32_t upto_page);

// ------------ Transmission of Records/Data ------------ //

// The format of record metadata.
// Defined by Glen.
typedef struct {
    // TODO
    uint32_t sequence_number;
    // datetime;
    // ???
    uint16_t payload_length;
} RecordHeader;

// TODO GPS STUFF ?
