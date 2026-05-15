// serial.h
//
// POSIX serial port wrapper (Mac/Linux). Uses termios under the hood.
// Thin layer with timeouts and clean error returns.

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Opaque handle. Internally just an fd, but callers shouldn't depend on that.
typedef int serial_handle_t;
#define SERIAL_INVALID (-1)

// Open a serial port at the given baud. Returns SERIAL_INVALID on failure.
// device: e.g. "/dev/cu.usbserial-0001" on Mac, "/dev/ttyUSB0" on Linux.
// baud: 9600, 115200, etc.
serial_handle_t serial_open(const char *device, uint32_t baud);

// Close the port. Safe to call on SERIAL_INVALID.
void serial_close(serial_handle_t h);

// Write all bytes. Returns number written (== len on success) or -1 on error.
// Blocks until all bytes are written or write fails.
int serial_write_all(serial_handle_t h, const uint8_t *data, size_t len);

// Read a single byte with timeout. Returns:
//   1 on success (byte read, written to *out)
//   0 on timeout
//  -1 on error
int serial_read_byte(serial_handle_t h, uint8_t *out, uint32_t timeout_ms);

// Discard any pending input bytes (e.g. after a reset, before a fresh command).
void serial_flush_input(serial_handle_t h);
