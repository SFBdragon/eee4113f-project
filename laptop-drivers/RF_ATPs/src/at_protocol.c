// at_protocol.c
//
// AT command sender/parser for Ai-Thinker LoRaWAN AT firmware.

// Feature test macros — platform-aware so we get clock_gettime on Linux
// without disabling BSD extensions on macOS.
#if defined(__linux__)
  #ifndef _DEFAULT_SOURCE
  #define _DEFAULT_SOURCE
  #endif
  #ifndef _POSIX_C_SOURCE
  #define _POSIX_C_SOURCE 200809L
  #endif
#elif defined(__APPLE__)
  #ifndef _DARWIN_C_SOURCE
  #define _DARWIN_C_SOURCE
  #endif
#endif

#include "../include/at_protocol.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Diagnostic mode: set LORA_DEBUG=1 in the environment to enable byte-level
// trace logging to stderr. Useful for diagnosing parser/protocol issues.
static int debug_enabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char *v = getenv("LORA_DEBUG");
        cached = (v != NULL && v[0] != '\0' && v[0] != '0') ? 1 : 0;
    }
    return cached;
}

// Return monotonic time in milliseconds. Used for deadline-based timeouts
// when we have to read multiple lines within a single overall timeout.
static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

at_result_t at_read_line(serial_handle_t h, char *out, size_t out_max, uint32_t timeout_ms) {
    if (out == NULL || out_max < 2) return AT_ERR_OVERFLOW;

    uint64_t deadline = now_ms() + timeout_ms;
    size_t idx = 0;

    while (1) {
        uint64_t t = now_ms();
        if (t >= deadline) return AT_ERR_TIMEOUT;
        uint32_t remaining = (uint32_t)(deadline - t);

        uint8_t b;
        int r = serial_read_byte(h, &b, remaining);
        if (r < 0) return AT_ERR_UART;
        if (r == 0) return AT_ERR_TIMEOUT;

        if (debug_enabled()) {
            fprintf(stderr, "[rx] 0x%02X %c\n", b,
                    (b >= 0x20 && b < 0x7F) ? b : '.');
        }

        // End of line: \n terminates. We strip trailing \r if present.
        if (b == '\n') {
            // Empty line? Skip it and keep reading.
            if (idx == 0) continue;
            if (idx > 0 && out[idx - 1] == '\r') idx--;
            out[idx] = '\0';

            // Some firmware emits prompt-like lines such as "ASR6501:~#" or
            // fragments of it (the module's line endings are inconsistent and
            // sometimes split the prompt across two reads).
            // Skip anything that looks like a prompt fragment.
            if (strstr(out, ":~#") != NULL ||
                strstr(out, "ASR") != NULL ||
                strstr(out, "~#") != NULL) {
                idx = 0;
                continue;
            }

            return AT_OK;
        }

        // Discard \r — handled at \n.
        if (b == '\r') continue;

        // Buffer the byte.
        if (idx + 1 >= out_max) {
            // Overflow: terminate and return what we have as an error.
            out[out_max - 1] = '\0';
            return AT_ERR_OVERFLOW;
        }
        out[idx++] = (char)b;
    }
}

// Compare line against expected string, ignoring leading whitespace.
static bool line_equals(const char *line, const char *expected) {
    while (*line == ' ' || *line == '\t') line++;
    return strcmp(line, expected) == 0;
}

// Check whether a line starts with the given prefix.
static bool line_starts_with(const char *line, const char *prefix) {
    return strncmp(line, prefix, strlen(prefix)) == 0;
}

at_result_t at_send_command(serial_handle_t h,
                            const char *cmd,
                            char *response, size_t response_max,
                            uint32_t timeout_ms) {
    if (h == SERIAL_INVALID || cmd == NULL) return AT_ERR_UART;

    // Flush any stale input first so we don't confuse it with our response.
    // Note: this discards any unsolicited messages that may have arrived.
    // Acceptable here because at_send_command is used for *commands*, not
    // for receiving packets. The recv path uses at_read_line directly.
    serial_flush_input(h);

    // Build the line. Note: we use \r (CR) only, not \r\n.
    // The Ai-Thinker LoRaWAN AT firmware treats \r as the command terminator
    // and interprets a following \n as a second (empty) command, which makes
    // the response stream confusing to parse. CoolTerm with "CR+LF" works
    // because of its line-buffered display, but for byte-level parsing CR-only
    // is more reliable. The module's own output lines are still \r\n terminated.
    char buf[AT_LINE_BUF_MAX];
    int n = snprintf(buf, sizeof(buf), "%s\r", cmd);
    if (n < 0 || (size_t)n >= sizeof(buf)) return AT_ERR_OVERFLOW;

    if (serial_write_all(h, (const uint8_t *)buf, (size_t)n) != n) {
        return AT_ERR_UART;
    }
    if (debug_enabled()) {
        fprintf(stderr, "[tx] %s", buf);  // buf already ends in \r\n
    }

    // Now read lines until we see OK or ERROR.
    if (response != NULL && response_max > 0) response[0] = '\0';
    size_t response_used = 0;

    uint64_t deadline = now_ms() + timeout_ms;

    while (1) {
        uint64_t t = now_ms();
        if (t >= deadline) return AT_ERR_TIMEOUT;
        uint32_t remaining = (uint32_t)(deadline - t);

        char line[AT_LINE_BUF_MAX];
        at_result_t r = at_read_line(h, line, sizeof(line), remaining);
        if (r == AT_ERR_TIMEOUT) return AT_ERR_TIMEOUT;
        if (r != AT_OK) return r;

        // The module echoes the command back as the first line.
        // Match exact, or starts-with, since echo formatting varies.
        if (strcmp(line, cmd) == 0) continue;
        if (strncmp(line, cmd, strlen(cmd)) == 0) continue;

        // Check for terminators.
        if (line_equals(line, "OK")) return AT_OK;

        if (line_starts_with(line, "ERROR") ||
            line_starts_with(line, "+CME ERROR") ||
            line_starts_with(line, "ERR+")) {
            return AT_ERR_RESPONSE;
        }

        // Otherwise it's intermediate output. Append to response buffer if provided.
        if (response != NULL && response_max > 0) {
            size_t llen = strlen(line);
            if (response_used + llen + 2 < response_max) {
                if (response_used > 0) {
                    response[response_used++] = '\n';
                }
                memcpy(response + response_used, line, llen);
                response_used += llen;
                response[response_used] = '\0';
            }
        }
    }
}

void hex_encode(const uint8_t *in, size_t len, char *out) {
    static const char digits[] = "0123456789ABCDEF";
    for (size_t i = 0; i < len; i++) {
        out[2 * i]     = digits[(in[i] >> 4) & 0xF];
        out[2 * i + 1] = digits[in[i] & 0xF];
    }
    out[2 * len] = '\0';
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

int hex_decode(const char *in, uint8_t *out, size_t out_max) {
    size_t count = 0;
    int high = -1;

    for (const char *p = in; *p != '\0'; p++) {
        int v = hex_nibble(*p);
        if (v < 0) continue;  // skip non-hex chars (whitespace, quotes, commas)
        if (high < 0) {
            high = v;
        } else {
            if (count >= out_max) return -1;  // overflow
            out[count++] = (uint8_t)((high << 4) | v);
            high = -1;
        }
    }
    // Odd number of hex digits = malformed
    if (high >= 0) return -1;
    return (int)count;
}
