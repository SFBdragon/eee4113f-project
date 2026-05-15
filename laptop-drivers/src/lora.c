// lora.c
//
// Implements the LoRa half of drivers.h for the Ai-Thinker Ra-07H module.
//
// PAYLOAD LIMITATION
// ==================
// The Ai-Thinker stock LoRaWAN AT firmware (SDK v4.3 / v1.0.0 Feb 2020) does
// not expose a command for sending arbitrary peer-to-peer payloads.
//
//   * AT+DTRX wraps payloads in LoRaWAN MAC framing; other end-devices reject
//     them as malformed downlinks (verified empirically).
//   * AT+CTX is a production test-mode command that transmits a firmware-
//     generated test pattern of the requested LENGTH. Caller-supplied bytes
//     are NOT placed on the air.
//   * AT+CRX is the matching test-mode receiver; emits "[N/M]Received: ..."
//     lines on every decoded packet, including RSSI and SNR.
//
// CTX and CRX each put the module into a one-way dead-loop state (per
// Ai-Thinker docs and community confirmation). The module CANNOT switch
// between TX and RX without a hardware reset. This driver therefore picks
// a role at init time and stays in it.
//
// ROLE SELECTION
// ==============
// Set the environment variable LORA_ROLE to "tx" or "rx" before calling
// initialize_lora_module. Default is "rx".
//
//   LORA_ROLE=tx ./test_lora ...   - module will TX only, recv will fail
//   LORA_ROLE=rx ./test_lora ...   - module will RX only, send will fail
//
// For the ATP03 range test, the buoy-side runs LORA_ROLE=tx, the ship-side
// runs LORA_ROLE=rx.
//
// CONFIGURATION
// =============
//   * Frequency: 869.525 MHz (EU868 RX2 default, legal in ZA)
//   * Data rate: DR 0  (SF12 / BW125 kHz, maximum range)
//   * TX power: index 0 (regional max, 14 dBm EU868)

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

#include "../include/drivers.h"
#include "../include/serial.h"
#include "../include/at_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static uint64_t now_ms_unix(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

// ---------------- Configuration ---------------- //

#define DEFAULT_DEVICE_MAC   "/dev/cu.usbserial-0001"
#define DEFAULT_DEVICE_LINUX "/dev/ttyUSB0"
#define SERIAL_BAUD 115200

#define RF_FREQ_HZ "869525000"
#define RF_DR      "0"
#define RF_TXPOWER "0"

// LoRaWAN ABP config. Required even for raw test mode because some commands
// only accept their full parameter set after JOINMODE/keys/etc. are set.
// Matches the sequence proven in CoolTerm.
#define DEVADDR  "01020304"
#define APPSKEY  "00000000000000000000000000000000"
#define NWKSKEY  "00000000000000000000000000000000"
#define FREQBANDMASK "0001"
#define CCLASS_VAL "2"

// Timeouts (ms).
#define INIT_CMD_TIMEOUT_MS  3000
#define SEND_TX_TIMEOUT_MS   15000  // SF12 packets are slow

// ---------------- Module state ---------------- //

typedef enum { ROLE_NONE = 0, ROLE_TX, ROLE_RX } lora_role_t;

static serial_handle_t g_serial = SERIAL_INVALID;
static lora_role_t     g_role   = ROLE_NONE;

// Last received packet's link quality. Exposed for path-loss measurement.
int g_last_rssi_dbm = 0;
int g_last_snr_db   = 0;

// ---------------- Helpers ---------------- //

static const char *resolve_device(void) {
    const char *env = getenv("LORA_SERIAL_DEVICE");
    if (env != NULL && env[0] != '\0') return env;
#ifdef __APPLE__
    return DEFAULT_DEVICE_MAC;
#else
    return DEFAULT_DEVICE_LINUX;
#endif
}

static lora_role_t resolve_role(void) {
    const char *env = getenv("LORA_ROLE");
    if (env == NULL) return ROLE_RX;  // default
    if (strcmp(env, "tx") == 0 || strcmp(env, "TX") == 0) return ROLE_TX;
    if (strcmp(env, "rx") == 0 || strcmp(env, "RX") == 0) return ROLE_RX;
    fprintf(stderr, "[lora] WARNING: unknown LORA_ROLE='%s', defaulting to rx\n", env);
    return ROLE_RX;
}

// Send an AT command, verbose logging for init-time sanity checking.
// Returns true on AT_OK.
static bool at_step(const char *cmd) {
    fprintf(stderr, "[lora init] > %s\n", cmd);
    at_result_t r = at_send_command(g_serial, cmd, NULL, 0, INIT_CMD_TIMEOUT_MS);
    if (r != AT_OK) {
        fprintf(stderr, "[lora init]   FAILED (err=%d)\n", r);
        return false;
    }
    fprintf(stderr, "[lora init]   OK\n");
    return true;
}

// Same as at_step but treats failure as non-fatal (logs and returns true).
// Used for commands whose setter rejects in-progress values but the resulting
// state is still acceptable (e.g. CDATARATE on an already-joined module).
static bool at_step_optional(const char *cmd) {
    fprintf(stderr, "[lora init] > %s (optional)\n", cmd);
    at_result_t r = at_send_command(g_serial, cmd, NULL, 0, INIT_CMD_TIMEOUT_MS);
    if (r != AT_OK) {
        fprintf(stderr, "[lora init]   non-fatal failure (err=%d), continuing\n", r);
        return true;  // non-fatal
    }
    fprintf(stderr, "[lora init]   OK\n");
    return true;
}

// Write a command that doesn't have a parseable OK terminator (the CTX/CRX
// test commands log status lines instead of returning OK). Just write the
// bytes and a short settle.
static bool write_raw_command(const char *cmd) {
    fprintf(stderr, "[lora init] > %s (no-OK command)\n", cmd);
    serial_flush_input(g_serial);
    char buf[80];
    int n = snprintf(buf, sizeof(buf), "%s\r", cmd);
    if (serial_write_all(g_serial, (const uint8_t *)buf, (size_t)n) != n) {
        fprintf(stderr, "[lora init]   UART write failed\n");
        return false;
    }
    usleep(200 * 1000);
    fprintf(stderr, "[lora init]   sent\n");
    return true;
}

// ---------------- Public API ---------------- //

bool is_lora_module_attached() {
    if (g_serial != SERIAL_INVALID) return true;

    serial_handle_t h = serial_open(resolve_device(), SERIAL_BAUD);
    if (h == SERIAL_INVALID) return false;

    usleep(200 * 1000);
    serial_write_all(h, (const uint8_t *)"\r", 1);
    usleep(300 * 1000);
    serial_flush_input(h);

    at_result_t r = at_send_command(h, "AT+CCLASS?", NULL, 0, 2000);
    serial_close(h);
    return (r == AT_OK);
}

Status initialize_lora_module() {
    if (g_serial != SERIAL_INVALID) {
        fprintf(stderr, "[lora init] already initialized\n");
        return STATUS_SUCCESS;
    }

    g_role = resolve_role();
    const char *role_str = (g_role == ROLE_TX) ? "TX" : "RX";
    const char *dev = resolve_device();

    fprintf(stderr, "[lora init] ============================================\n");
    fprintf(stderr, "[lora init] role:    %s\n", role_str);
    fprintf(stderr, "[lora init] device:  %s\n", dev);
    fprintf(stderr, "[lora init] freq:    %s Hz\n", RF_FREQ_HZ);
    fprintf(stderr, "[lora init] dr:      %s (SF12/BW125)\n", RF_DR);
    fprintf(stderr, "[lora init] txpower: %s (regional max)\n", RF_TXPOWER);
    fprintf(stderr, "[lora init] ============================================\n");

    g_serial = serial_open(dev, SERIAL_BAUD);
    if (g_serial == SERIAL_INVALID) {
        fprintf(stderr, "[lora init] FAILED to open serial port\n");
        return STATUS_MODULE_DETACHED;
    }
    fprintf(stderr, "[lora init] serial port open\n");

    // Warmup: terminate any leftover partial command from a prior session.
    usleep(200 * 1000);
    serial_write_all(g_serial, (const uint8_t *)"\r", 1);
    usleep(300 * 1000);
    serial_flush_input(g_serial);
    fprintf(stderr, "[lora init] warmup done\n");

    // Full LoRaWAN ABP configuration sequence (matches the sequence proven
    // to work in CoolTerm). The Ai-Thinker firmware uses Class C continuous
    // RX as the basis for both CTX and CRX test modes; without this prelude
    // the module may not be in the expected radio state.
    bool ok = true;
    ok &= at_step("AT+CJOINMODE=1");
    ok &= at_step("AT+CDEVADDR=" DEVADDR);
    ok &= at_step("AT+CAPPSKEY=" APPSKEY);
    ok &= at_step("AT+CNWKSKEY=" NWKSKEY);
    ok &= at_step("AT+CFREQBANDMASK=" FREQBANDMASK);
    // CDATARATE setter rejects values in this firmware once joined; harmless.
    at_step_optional("AT+CDATARATE=" RF_DR);
    // CTXP setter also rejects in some module states; default power is fine.
    at_step_optional("AT+CTXP=" RF_TXPOWER);
    ok &= at_step("AT+CCLASS=" CCLASS_VAL);

    if (!ok) {
        fprintf(stderr, "[lora init] configuration sequence FAILED\n");
        serial_close(g_serial);
        g_serial = SERIAL_INVALID;
        return STATUS_MODULE_DETACHED;
    }
    fprintf(stderr, "[lora init] LoRaWAN ABP config complete\n");

    // Role-specific final step.
    if (g_role == ROLE_RX) {
        // Enter raw RX mode. From now on the module will emit
        // "[N/M]Received: ..." lines for every decoded packet.
        // Note: this is a one-way mode — calling send_lora_packet will fail.
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "AT+CRX=%s,%s", RF_FREQ_HZ, RF_DR);
        if (!write_raw_command(cmd)) {
            serial_close(g_serial);
            g_serial = SERIAL_INVALID;
            return STATUS_MODULE_DETACHED;
        }
        fprintf(stderr, "[lora init] entered raw RX mode\n");
    } else {
        // ROLE_TX: do NOT enter CRX. Leave the module ready for AT+CTX.
        fprintf(stderr, "[lora init] module is in TX role, ready for AT+CTX\n");
    }

    fprintf(stderr, "[lora init] initialization complete\n");
    fprintf(stderr, "[lora init] ============================================\n");
    return STATUS_SUCCESS;
}

void shutdown_lora_module() {
    if (g_serial != SERIAL_INVALID) {
        serial_close(g_serial);
        g_serial = SERIAL_INVALID;
        g_role = ROLE_NONE;
    }
}

double get_lora_byterate() {
    // DR0 / SF12 / BW125 / CR4-5 → ~293 bps → ~36.6 bytes/s
    return 36.6;
}

Status send_lora_packet(uint8_t *data, BufLen len) {
    if (g_serial == SERIAL_INVALID) {
        fprintf(stderr, "[lora send] module not initialized\n");
        return STATUS_MODULE_DETACHED;
    }
    if (g_role != ROLE_TX) {
        fprintf(stderr, "[lora send] module is in RX role, cannot send. "
                        "Set LORA_ROLE=tx before init.\n");
        return -1;
    }
    if (len == 0 || len > MAX_LORA_SEND_PACKET_LEN) return -1;

    // PAYLOAD LIMITATION: data content is NOT transmitted on this firmware.
    // The on-air bytes are a firmware-generated test pattern of length `len`.
    (void)data;

    char cmd[80];
    int n = snprintf(cmd, sizeof(cmd), "AT+CTX=%s,%s,%s,%u",
                     RF_FREQ_HZ, RF_DR, RF_TXPOWER, (unsigned)len);
    if (n < 0 || (size_t)n >= sizeof(cmd)) return -1;

    serial_flush_input(g_serial);

    char line[AT_LINE_BUF_MAX];
    int written = snprintf(line, sizeof(line), "%s\r", cmd);
    if (written < 0 || (size_t)written >= sizeof(line)) return -1;
    if (serial_write_all(g_serial, (const uint8_t *)line, (size_t)written) != written) {
        return -1;
    }

    // Wait for "OnTxDone" within the overall TX timeout.
    // Each AT+CTX call invokes one test-pattern transmission and then
    // (per the firmware) re-arms for the next call. We do NOT call CRX
    // afterwards — that would put the module into the dead-loop RX state.
    uint64_t deadline = now_ms_unix() + SEND_TX_TIMEOUT_MS;
    while (now_ms_unix() < deadline) {
        uint32_t remaining = (uint32_t)(deadline - now_ms_unix());
        at_result_t r = at_read_line(g_serial, line, sizeof(line), remaining);
        if (r == AT_ERR_TIMEOUT) break;
        if (r != AT_OK) continue;

        if (strncmp(line, cmd, strlen(cmd)) == 0) continue;  // echo

        if (strstr(line, "OnTxDone") != NULL) {
            return STATUS_SUCCESS;
        }
        if (strncmp(line, "ERR", 3) == 0 ||
            strncmp(line, "+CME ERROR", 10) == 0) {
            fprintf(stderr, "[lora send] module returned error: %s\n", line);
            return -1;
        }
        // log line — keep reading.
    }
    fprintf(stderr, "[lora send] timeout waiting for OnTxDone\n");
    return -1;
}

Status recv_lora_packet(uint8_t data[MAX_LORA_RECV_PACKET_LEN], BufLen *len, uint32_t timeout_ms) {
    if (g_serial == SERIAL_INVALID) return STATUS_MODULE_DETACHED;
    if (g_role != ROLE_RX) {
        fprintf(stderr, "[lora recv] module is in TX role, cannot receive. "
                        "Set LORA_ROLE=rx before init.\n");
        return -1;
    }
    if (data == NULL || len == NULL) return -1;

    // Module is already in CRX mode (entered by initialize_lora_module).
    // Read lines until we see "Received:" or timeout.
    char line[AT_LINE_BUF_MAX];
    uint64_t deadline = now_ms_unix() + timeout_ms;

    while (now_ms_unix() < deadline) {
        uint32_t remaining = (uint32_t)(deadline - now_ms_unix());
        at_result_t r = at_read_line(g_serial, line, sizeof(line), remaining);
        if (r == AT_ERR_TIMEOUT) return STATUS_RECEIVE_TIMEOUT;
        if (r != AT_OK) continue;

        char *recv_marker = strstr(line, "Received:");
        if (recv_marker == NULL) continue;

        // Parse payload (firmware test counter, an ASCII integer).
        const char *payload_str = recv_marker + strlen("Received:");
        while (*payload_str == ' ') payload_str++;

        char payload_buf[32];
        size_t pi = 0;
        while (*payload_str != '\0' && *payload_str != ',' &&
               pi + 1 < sizeof(payload_buf)) {
            payload_buf[pi++] = *payload_str++;
        }
        payload_buf[pi] = '\0';

        // Extract RSSI and SNR.
        int rssi = 0, snr = 0;
        const char *rssi_str = strstr(line, "rssi");
        const char *snr_str  = strstr(line, "snr");
        if (rssi_str) sscanf(rssi_str, "rssi = %d", &rssi);
        if (snr_str)  sscanf(snr_str,  "snr = %d",  &snr);

        g_last_rssi_dbm = rssi;
        g_last_snr_db   = snr;

        fprintf(stderr, "[lora recv] payload=\"%s\" rssi=%d snr=%d\n",
                payload_buf, rssi, snr);

        size_t copy_len = pi;
        if (copy_len > MAX_LORA_RECV_PACKET_LEN) copy_len = MAX_LORA_RECV_PACKET_LEN;
        memcpy(data, payload_buf, copy_len);
        *len = (BufLen)copy_len;

        return STATUS_SUCCESS;
    }
    return STATUS_RECEIVE_TIMEOUT;
}
