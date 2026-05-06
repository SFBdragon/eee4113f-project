// transmission.h

#include <cstdint>
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

// Return the number of milliseconds from some arbitrary fixed epoch (e.g. since startup).
// Defined by Glen. Called by Shaun for timing information.
uint32_t get_time_since_epoch_ms();

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
#define STORAGE_PAGE_SIZE /* = <glen plz specify> */

// The total number of pages available for record storage.
uint32_t storage_total_pages();

uint32_t storage_first_readable_page();

uint32_t storage_first_protected_page();

uint32_t storage_last_readable_page();

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
} Policy;

// The end-user has chosen an overwriting policy to take effect from now on.
void set_overwrite_policy(Policy policy);

// The end-user has decided that the records in pages from
// `storage_first_protected_page` (inclusive) to `upto_page` (exclusive)
// no longer need to be preseved on telemeter storage.
// This should not need to immediately wipe any data.
// This should have an effect regardless of the current overwrite policy.
//
// This broadly indicates that the user has a copy of this data or no longer
// needs the data and it can be overwritten if needed regardless of policy.
// This should adjust `storage_first_protected_page` but not
// `storage_first_readable_page` or `storage_last_readable_page`.
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

// The following two are defined by Shaun and called by Glen.

// Whether incoming records should be passed to `stream_record` as well as stored.
bool should_stream_record();

// Called by Glen when `should_stream_record` and a new record is received.
int stream_record(RecordHeader *header, uint8_t* payload);


// ------------ Measurement System Configuration ------------ //

// Parameter type values. Note that any value can be readonly.
// This prevents the user of the control panel from setting a value.
// This is 
#define PARAM_READONLY 0b10000000
#define PARAM_UNSIGNED_INT32 1
#define PARAM_SIGNED_INT32 2
#define PARAM_FLOAT32 3
#define PARAM_CHECKBOX_BOOL 4
#define PARAM_UTF8_STRING 5
#define PARAM_BUTTON_PRESS 6

typedef struct {
    union {
        uint32_t unsigned_int32;
        int32_t signed_int32;
        float float32;
        bool checkbox_bool;
        struct { uint16_t len; uint8_t data0; } *utf8_string;
        struct { } button_press;

    } value;
    uint8_t type;
    uint8_t name[8];
} ConfigurableParameter;

// Get a buffer to all the configurable parameters that should be advertized to the control panel.
// `value` contains the current/default value that should be displayed to the user.
// Defined by Glen. Called by Shaun. Shaun will not modify or free `params`.
void get_configurable_parameters(ConfigurableParameter *params, uint16_t count);


// The user has entered a value for a particular parameter, this should be sent to the measurement system.
// Defined by Glen. Called by Shaun. Shaun will cleanup the data after the function returns.
void set_configurable_parameter(ConfigurableParameter *set);




// TODO GPS STUFF ?