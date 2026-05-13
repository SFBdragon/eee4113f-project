// at_protocol.h
//
// AT command layer for Ai-Thinker Ra-07H LoRaWAN AT firmware.
// Handles line-based reading, command response parsing, and hex encoding.

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "serial.h"

// Maximum length of a single line we'll buffer from the module.
// Lines like "+DRX:64,<128 hex chars>" comfortably fit in 256.
#define AT_LINE_BUF_MAX 512

// Result codes from at_send_command. Negative = problem.
typedef enum {
    AT_OK = 0,
    AT_ERR_TIMEOUT = -1,
    AT_ERR_RESPONSE = -2,    // module returned ERROR or unexpected response
    AT_ERR_UART = -3,        // serial read/write failure
    AT_ERR_OVERFLOW = -4,    // response longer than buffer
} at_result_t;

// Read one CRLF-terminated line from the module, with timeout.
// Strips trailing \r\n. Returns AT_OK with line in `out` (null-terminated),
// or AT_ERR_TIMEOUT if no complete line arrives within timeout_ms.
// Skips empty lines automatically (the module emits plenty of them).
at_result_t at_read_line(serial_handle_t h, char *out, size_t out_max, uint32_t timeout_ms);

// Send an AT command (without \r\n — this function appends it).
// Waits for "OK" or "ERROR" response, with timeout.
// Any intermediate lines are written into `response` (concatenated, newline-separated)
// if response != NULL. Pass NULL if you don't care about intermediate output.
//
// Returns AT_OK if module replied OK, AT_ERR_RESPONSE if it replied ERROR,
// AT_ERR_TIMEOUT if no terminating OK/ERROR arrived in time.
//
// IMPORTANT: this function will discard any unsolicited messages (e.g. +DRX:)
// that arrive while waiting. For receive logic, use at_read_line directly.
at_result_t at_send_command(serial_handle_t h,
                            const char *cmd,
                            char *response, size_t response_max,
                            uint32_t timeout_ms);

// Convert bytes to a hex string (uppercase, no separators).
// out must have room for 2*len+1 chars (including null terminator).
void hex_encode(const uint8_t *in, size_t len, char *out);

// Parse a hex string into bytes. Returns number of bytes decoded, or -1 on parse error.
// Ignores any non-hex chars (so leading/trailing whitespace, quotes are tolerated).
int hex_decode(const char *in, uint8_t *out, size_t out_max);
