// wifi.c
//
// Implementation of the WiFi half of drivers.h, backed by an ESP32
// running ship_wifi.ino or buoy_wifi.ino as a "smart modem".
//
// The driver opens a UART to the ESP32 and speaks the framed binary
// protocol defined in wifi_protocol.h. A background thread reads
// incoming frames so recv_wifi_packet can block and so MSG_RECV frames
// arriving while a send is in flight don't get lost.
//
// Role selection (ship vs buoy) is set via the WIFI_ROLE env var.
//   WIFI_ROLE=ship  - the ESP32 is running ship_wifi.ino (AP side)
//   WIFI_ROLE=buoy  - the ESP32 is running buoy_wifi.ino (STA side)
// Both roles use the same UART protocol; the difference is only that
// initialize_wifi_module sends MSG_INIT to the buoy to trigger its
// Wi-Fi connection attempt.

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
#include "../include/wifi_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

// ---------------- Configuration ---------------- //

#define DEFAULT_DEVICE_MAC   "/dev/cu.usbserial-0001"
#define DEFAULT_DEVICE_LINUX "/dev/ttyUSB0"

#define INIT_TIMEOUT_MS    20000  // Wi-Fi association can take a while
#define SEND_TIMEOUT_MS     5000

// Circular buffer of received packets. recv_wifi_packet blocks until non-empty.
#define RECV_QUEUE_DEPTH 32

typedef struct {
    uint64_t srcmac;
    BufLen   len;
    uint8_t  data[MAX_WIFI_RECV_PACKET_LEN];
} recv_slot_t;

typedef enum { ROLE_NONE = 0, ROLE_SHIP, ROLE_BUOY } wifi_role_t;

// ---------------- Module state ---------------- //

static serial_handle_t g_serial = SERIAL_INVALID;
static wifi_role_t     g_role   = ROLE_NONE;
static uint8_t         g_self_mac[6] = {0};
static bool            g_initialized = false;

// Reader thread
static pthread_t       g_reader_thread;
static volatile bool   g_reader_run = false;

// Recv queue
static recv_slot_t     g_recv_queue[RECV_QUEUE_DEPTH];
static int             g_recv_head = 0;
static int             g_recv_tail = 0;
static int             g_recv_count = 0;
static pthread_mutex_t g_recv_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_recv_cond  = PTHREAD_COND_INITIALIZER;

// Send-response signalling (ACK/NACK/STATUS)
static pthread_mutex_t g_resp_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_resp_cond  = PTHREAD_COND_INITIALIZER;
static volatile bool   g_resp_pending = false;
static volatile uint8_t g_resp_type = 0;
static uint8_t         g_resp_payload[WIFI_MAX_PAYLOAD];
static uint16_t        g_resp_payload_len = 0;

// READY signal
static volatile bool   g_ready_seen = false;
static pthread_mutex_t g_ready_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_ready_cond  = PTHREAD_COND_INITIALIZER;

// Asynchronous NACK counter. Incremented by the reader thread whenever
// a WIFI_MSG_NACK frame arrives, regardless of whether anyone is waiting
// on a response. Lets the throughput test (which uses fire-and-forget
// sends) detect server-side rejections it would otherwise miss.
//
// Reads/writes use GCC/Clang __atomic builtins instead of <stdatomic.h>
// (which needs C11) to stay C99-compatible while remaining lock-free.
static volatile uint64_t g_nack_count = 0;

// Cumulative count of MSG_RECV frames the reader thread has seen.
// Includes ones dropped due to recv queue overflow. Useful for
// diagnosing whether RECV frames are reaching the laptop at all
// independently of whether run_recv is draining the queue fast enough.
static volatile uint64_t g_recv_frame_count = 0;

// Cumulative bytes seen in MSG_RECV payloads (excluding the 6-byte
// MAC prefix). Lets the test compare "reader thread saw N bytes"
// with "test code dequeued M bytes" to find pipeline stalls.
static volatile uint64_t g_recv_bytes_count = 0;

// ---------------- Helpers ---------------- //

static uint64_t now_ms_mono(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

static const char *resolve_device(void) {
    const char *env = getenv("WIFI_SERIAL_DEVICE");
    if (env && env[0]) return env;
#ifdef __APPLE__
    return DEFAULT_DEVICE_MAC;
#else
    return DEFAULT_DEVICE_LINUX;
#endif
}

static wifi_role_t resolve_role(void) {
    const char *env = getenv("WIFI_ROLE");
    if (!env) return ROLE_SHIP;
    if (strcmp(env, "buoy") == 0 || strcmp(env, "BUOY") == 0) return ROLE_BUOY;
    if (strcmp(env, "ship") == 0 || strcmp(env, "SHIP") == 0) return ROLE_SHIP;
    fprintf(stderr, "[wifi] unknown WIFI_ROLE='%s', defaulting to ship\n", env);
    return ROLE_SHIP;
}

static void mac_u64_to_bytes(uint64_t v, uint8_t *out) {
    for (int i = 5; i >= 0; i--) { out[i] = (uint8_t)(v & 0xFF); v >>= 8; }
}

// Build & send a frame on the UART.
static int send_frame(uint8_t type, const uint8_t *payload, uint16_t len) {
    if (g_serial == SERIAL_INVALID) return -1;
    if (len > WIFI_MAX_PAYLOAD) return -1;

    uint8_t hdr[5];
    hdr[0] = WIFI_FRAME_SYNC0;
    hdr[1] = WIFI_FRAME_SYNC1;
    hdr[2] = type;
    hdr[3] = (uint8_t)(len >> 8);
    hdr[4] = (uint8_t)(len & 0xFF);

    // CRC over type+lenhi+lenlo+payload
    uint8_t cb[3] = { type, hdr[3], hdr[4] };
    uint16_t crc = wifi_crc16(cb, 3);
    // Continue CRC over payload
    if (payload && len) {
        for (uint16_t i = 0; i < len; i++) {
            crc ^= ((uint16_t)payload[i]) << 8;
            for (int b = 0; b < 8; b++) {
                if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
                else              crc = (uint16_t)(crc << 1);
            }
        }
    }

    if (serial_write_all(g_serial, hdr, 5) != 5) return -1;
    if (payload && len) {
        if (serial_write_all(g_serial, payload, len) != (int)len) return -1;
    }
    uint8_t tail[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
    if (serial_write_all(g_serial, tail, 2) != 2) return -1;
    return 0;
}

// ---------------- Reader thread ---------------- //

static void deliver_recv(const uint8_t *payload, uint16_t len) {
    if (len < 6) return;
    // Count the frame and bytes as seen by the reader thread, BEFORE
    // checking queue overflow. This way g_recv_frame_count reflects
    // every RECV frame that arrived from the ESP32, even ones we had
    // to drop. Compare with the test's dequeued count to find stalls.
    uint16_t dlen_check = len - 6;
    __atomic_fetch_add(&g_recv_frame_count, 1, __ATOMIC_RELAXED);
    __atomic_fetch_add(&g_recv_bytes_count, dlen_check, __ATOMIC_RELAXED);

    pthread_mutex_lock(&g_recv_mutex);
    if (g_recv_count >= RECV_QUEUE_DEPTH) {
        // drop oldest
        g_recv_head = (g_recv_head + 1) % RECV_QUEUE_DEPTH;
        g_recv_count--;
    }
    recv_slot_t *slot = &g_recv_queue[g_recv_tail];
    uint64_t mac = 0;
    for (int i = 0; i < 6; i++) mac = (mac << 8) | payload[i];
    slot->srcmac = mac;
    uint16_t dlen = len - 6;
    if (dlen > MAX_WIFI_RECV_PACKET_LEN) dlen = MAX_WIFI_RECV_PACKET_LEN;
    memcpy(slot->data, payload + 6, dlen);
    slot->len = (BufLen)dlen;
    g_recv_tail = (g_recv_tail + 1) % RECV_QUEUE_DEPTH;
    g_recv_count++;
    pthread_cond_signal(&g_recv_cond);
    pthread_mutex_unlock(&g_recv_mutex);
}

static void deliver_response(uint8_t type, const uint8_t *payload, uint16_t len) {
    if (type == WIFI_MSG_NACK) {
        // Count every NACK, even ones nobody is waiting for. This is the
        // only way the fire-and-forget send path can learn that the ESP32
        // rejected a frame.
        __atomic_fetch_add(&g_nack_count, 1, __ATOMIC_RELAXED);
    }
    pthread_mutex_lock(&g_resp_mutex);
    g_resp_type = type;
    if (len > sizeof(g_resp_payload)) len = sizeof(g_resp_payload);
    memcpy(g_resp_payload, payload, len);
    g_resp_payload_len = len;
    g_resp_pending = true;
    pthread_cond_signal(&g_resp_cond);
    pthread_mutex_unlock(&g_resp_mutex);
}

static void deliver_ready(const uint8_t *payload, uint16_t len) {
    if (len >= 6) memcpy(g_self_mac, payload, 6);
    pthread_mutex_lock(&g_ready_mutex);
    g_ready_seen = true;
    pthread_cond_broadcast(&g_ready_cond);
    pthread_mutex_unlock(&g_ready_mutex);
}

static void *reader_thread_fn(void *arg) {
    (void)arg;
    enum { S0, S1, ST, SL1, SL2, SP, SC1, SC2 } state = S0;
    uint8_t type = 0;
    uint16_t len = 0;
    uint16_t pi = 0;
    uint16_t crc_rx = 0;
    static uint8_t pbuf[WIFI_MAX_PAYLOAD + 16];

    while (g_reader_run) {
        uint8_t b;
        int r = serial_read_byte(g_serial, &b, 100);
        if (r < 0) continue;
        if (r == 0) continue;

        switch (state) {
            case S0:
                if (b == WIFI_FRAME_SYNC0) state = S1;
                break;
            case S1:
                if (b == WIFI_FRAME_SYNC1) state = ST;
                else if (b == WIFI_FRAME_SYNC0) state = S1;
                else state = S0;
                break;
            case ST: type = b; state = SL1; break;
            case SL1: len = ((uint16_t)b) << 8; state = SL2; break;
            case SL2:
                len |= b;
                pi = 0;
                if (len > WIFI_MAX_PAYLOAD) { state = S0; break; }
                state = (len == 0) ? SC1 : SP;
                break;
            case SP:
                pbuf[pi++] = b;
                if (pi == len) state = SC1;
                break;
            case SC1: crc_rx = ((uint16_t)b) << 8; state = SC2; break;
            case SC2: {
                crc_rx |= b;
                uint8_t cb[3] = { type, (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
                uint16_t crc = wifi_crc16(cb, 3);
                for (uint16_t i = 0; i < len; i++) {
                    crc ^= ((uint16_t)pbuf[i]) << 8;
                    for (int bb = 0; bb < 8; bb++) {
                        if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
                        else              crc = (uint16_t)(crc << 1);
                    }
                }
                if (crc == crc_rx) {
                    // Dispatch
                    switch (type) {
                        case WIFI_MSG_RECV:
                            deliver_recv(pbuf, len);
                            break;
                        case WIFI_MSG_ACK:
                        case WIFI_MSG_NACK:
                        case WIFI_MSG_STATUS:
                            deliver_response(type, pbuf, len);
                            break;
                        case WIFI_MSG_READY:
                            deliver_ready(pbuf, len);
                            break;
                        case WIFI_MSG_LOG:
                            fprintf(stderr, "[wifi esp32] ");
                            fwrite(pbuf, 1, len, stderr);
                            fputc('\n', stderr);
                            break;
                        default:
                            fprintf(stderr, "[wifi] unknown frame type 0x%02x\n", type);
                            break;
                    }
                } else {
                    fprintf(stderr, "[wifi] bad CRC on incoming frame\n");
                }
                state = S0;
                break;
            }
        }
    }
    return NULL;
}

// Wait for an ACK/NACK/STATUS response, with timeout.
// Returns the type seen (0x81/0x82/0x84) or 0 on timeout.
static uint8_t wait_response(uint32_t timeout_ms,
                             uint8_t *out_payload, uint16_t *out_len) {
    uint64_t deadline = now_ms_mono() + timeout_ms;
    pthread_mutex_lock(&g_resp_mutex);
    while (!g_resp_pending) {
        uint64_t now = now_ms_mono();
        if (now >= deadline) { pthread_mutex_unlock(&g_resp_mutex); return 0; }
        uint64_t rem = deadline - now;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += rem / 1000;
        ts.tv_nsec += (rem % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        int r = pthread_cond_timedwait(&g_resp_cond, &g_resp_mutex, &ts);
        if (r == ETIMEDOUT) { pthread_mutex_unlock(&g_resp_mutex); return 0; }
    }
    uint8_t type = g_resp_type;
    if (out_payload && out_len) {
        memcpy(out_payload, g_resp_payload, g_resp_payload_len);
        *out_len = g_resp_payload_len;
    }
    g_resp_pending = false;
    pthread_mutex_unlock(&g_resp_mutex);
    return type;
}

static bool wait_ready(uint32_t timeout_ms) {
    uint64_t deadline = now_ms_mono() + timeout_ms;
    pthread_mutex_lock(&g_ready_mutex);
    while (!g_ready_seen) {
        uint64_t now = now_ms_mono();
        if (now >= deadline) { pthread_mutex_unlock(&g_ready_mutex); return false; }
        uint64_t rem = deadline - now;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += rem / 1000;
        ts.tv_nsec += (rem % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        int r = pthread_cond_timedwait(&g_ready_cond, &g_ready_mutex, &ts);
        if (r == ETIMEDOUT) { pthread_mutex_unlock(&g_ready_mutex); return false; }
    }
    pthread_mutex_unlock(&g_ready_mutex);
    return true;
}

// ---------------- Public API (drivers.h) ---------------- //

bool is_wifi_module_attached(void) {
    const char *dev = resolve_device();
    serial_handle_t h = serial_open(dev, WIFI_UART_BAUD);
    if (h == SERIAL_INVALID) return false;
    serial_close(h);
    return true;
}

Status initialize_wifi_module(void) {
    if (g_initialized) return STATUS_SUCCESS;

    g_role = resolve_role();
    const char *dev = resolve_device();

    fprintf(stderr, "[wifi init] ===========================================\n");
    fprintf(stderr, "[wifi init] role:   %s\n", g_role == ROLE_SHIP ? "ship (AP)" : "buoy (STA)");
    fprintf(stderr, "[wifi init] device: %s\n", dev);
    fprintf(stderr, "[wifi init] baud:   %u\n", WIFI_UART_BAUD);
    fprintf(stderr, "[wifi init] ===========================================\n");

    g_serial = serial_open(dev, WIFI_UART_BAUD);
    if (g_serial == SERIAL_INVALID) {
        fprintf(stderr, "[wifi init] failed to open serial port\n");
        return STATUS_MODULE_DETACHED;
    }

    // Reset reader state
    g_ready_seen = false;
    g_resp_pending = false;
    g_recv_head = g_recv_tail = g_recv_count = 0;
    __atomic_store_n(&g_nack_count, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_recv_frame_count, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_recv_bytes_count, 0, __ATOMIC_RELAXED);

    g_reader_run = true;
    if (pthread_create(&g_reader_thread, NULL, reader_thread_fn, NULL) != 0) {
        fprintf(stderr, "[wifi init] failed to spawn reader thread\n");
        serial_close(g_serial); g_serial = SERIAL_INVALID;
        return STATUS_MODULE_DETACHED;
    }

    // Wait for READY frame from ESP32 (fires on its boot)
    fprintf(stderr, "[wifi init] waiting for READY...\n");
    if (!wait_ready(8000)) {
        // ESP32 may have booted before we attached; force one via INIT below
        fprintf(stderr, "[wifi init] no READY in 8s; will try sending INIT anyway\n");
    } else {
        fprintf(stderr, "[wifi init] received READY, self MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                g_self_mac[0], g_self_mac[1], g_self_mac[2],
                g_self_mac[3], g_self_mac[4], g_self_mac[5]);
    }

    // Send INIT — ship side ACKs immediately, buoy side triggers Wi-Fi connect
    fprintf(stderr, "[wifi init] sending INIT\n");
    if (send_frame(WIFI_MSG_INIT, NULL, 0) != 0) {
        fprintf(stderr, "[wifi init] failed to send INIT\n");
        g_reader_run = false; pthread_join(g_reader_thread, NULL);
        serial_close(g_serial); g_serial = SERIAL_INVALID;
        return STATUS_MODULE_DETACHED;
    }

    uint8_t resp_pl[WIFI_MAX_PAYLOAD]; uint16_t resp_len = 0;
    uint8_t resp = wait_response(INIT_TIMEOUT_MS, resp_pl, &resp_len);
    if (resp == WIFI_MSG_ACK) {
        fprintf(stderr, "[wifi init] INIT acknowledged\n");
        g_initialized = true;
        return STATUS_SUCCESS;
    } else if (resp == WIFI_MSG_NACK) {
        fprintf(stderr, "[wifi init] INIT NACKed (reason 0x%02x)\n",
                resp_len > 0 ? resp_pl[0] : 0);
    } else {
        fprintf(stderr, "[wifi init] no response to INIT (timeout)\n");
    }
    g_reader_run = false; pthread_join(g_reader_thread, NULL);
    serial_close(g_serial); g_serial = SERIAL_INVALID;
    return STATUS_MODULE_DETACHED;
}

void shutdown_wifi_module(void) {
    if (!g_initialized) return;
    send_frame(WIFI_MSG_SHUTDOWN, NULL, 0);
    uint8_t pl[WIFI_MAX_PAYLOAD]; uint16_t pl_len;
    wait_response(2000, pl, &pl_len);

    g_reader_run = false;
    pthread_join(g_reader_thread, NULL);
    serial_close(g_serial);
    g_serial = SERIAL_INVALID;
    g_initialized = false;
}

Status send_wifi_packet(uint64_t destmac, uint8_t *data, BufLen len) {
    if (!g_initialized) return STATUS_MODULE_DETACHED;
    if (len > MAX_WIFI_SEND_PACKET_LEN) return -1;

    // Discard any stale response left over from a previous call that
    // timed out or was abandoned. Without this, a late NACK could be
    // matched against this send's wait_response.
    pthread_mutex_lock(&g_resp_mutex);
    g_resp_pending = false;
    pthread_mutex_unlock(&g_resp_mutex);

    uint8_t frame[6 + MAX_WIFI_SEND_PACKET_LEN];
    mac_u64_to_bytes(destmac, frame);
    memcpy(frame + 6, data, len);

    if (send_frame(WIFI_MSG_SEND, frame, 6 + len) != 0) return -1;

    uint8_t pl[WIFI_MAX_PAYLOAD]; uint16_t pl_len;
    uint8_t resp = wait_response(SEND_TIMEOUT_MS, pl, &pl_len);
    if (resp == WIFI_MSG_ACK) return STATUS_SUCCESS;
    return -1;
}

// Fire-and-forget send. Writes the frame to the UART and returns as soon
// as the bytes are accepted by the kernel; does NOT wait for an ACK from
// the ESP32. Reliability is delegated to TCP end-to-end (the SEND frame
// either makes it onto the TCP socket or doesn't; TCP guarantees ordered
// delivery from there).
//
// This is the path used by the throughput test (run_send). Without it,
// per-frame ACK round-trips through the macOS USB-serial driver cap
// throughput at ~100 frames/sec regardless of frame size.
//
// Returns STATUS_SUCCESS if the UART write completed.
// Returns -1 if the driver isn't initialized or the write failed.
//
// NACKs that occur asynchronously can be counted via wifi_get_nack_count().
Status send_wifi_packet_nowait(uint64_t destmac, const uint8_t *data, BufLen len) {
    if (!g_initialized) return STATUS_MODULE_DETACHED;
    if (len > MAX_WIFI_SEND_PACKET_LEN) return -1;

    uint8_t frame[6 + MAX_WIFI_SEND_PACKET_LEN];
    mac_u64_to_bytes(destmac, frame);
    memcpy(frame + 6, data, len);

    if (send_frame(WIFI_MSG_SEND, frame, 6 + len) != 0) return -1;
    return STATUS_SUCCESS;
}

// Returns the total number of NACK frames received since the last
// initialize_wifi_module() call. Safe to call from any thread.
uint64_t wifi_get_nack_count(void) {
    return __atomic_load_n(&g_nack_count, __ATOMIC_RELAXED);
}

// Returns the total number of MSG_RECV frames the reader thread has
// seen from the ESP32 (including any dropped due to queue overflow).
// Useful for distinguishing "ESP32 not forwarding" from "test code
// not draining fast enough."
uint64_t wifi_get_recv_frame_count(void) {
    return __atomic_load_n(&g_recv_frame_count, __ATOMIC_RELAXED);
}

// Cumulative bytes seen in MSG_RECV payloads (data portion only,
// excluding the 6-byte MAC prefix).
uint64_t wifi_get_recv_bytes_count(void) {
    return __atomic_load_n(&g_recv_bytes_count, __ATOMIC_RELAXED);
}

// Wait for the ESP32 pipeline to drain by issuing a synchronous status
// query. Because the ESP32 processes UART frames in order, when we get
// the STATUS reply back we know every preceding SEND frame has been
// fully written to TCP. Use this at the end of a no-wait send run so
// throughput math reflects bytes actually delivered, not bytes queued.
//
// Returns STATUS_SUCCESS if the status reply arrived within timeout_ms,
// or -1 on timeout. Callers can treat timeout as a soft warning rather
// than a hard failure.
Status wifi_drain_pending(uint32_t timeout_ms) {
    if (!g_initialized) return STATUS_MODULE_DETACHED;

    pthread_mutex_lock(&g_resp_mutex);
    g_resp_pending = false;
    pthread_mutex_unlock(&g_resp_mutex);

    if (send_frame(WIFI_MSG_STATUS_Q, NULL, 0) != 0) return -1;

    uint8_t pl[WIFI_MAX_PAYLOAD]; uint16_t pl_len = 0;
    uint8_t resp = wait_response(timeout_ms, pl, &pl_len);
    if (resp == WIFI_MSG_STATUS || resp == WIFI_MSG_ACK) return STATUS_SUCCESS;
    return -1;
}

Status recv_wifi_packet(uint64_t *srcmac,
                        uint8_t data[MAX_WIFI_RECV_PACKET_LEN],
                        BufLen *len) {
    if (!g_initialized) return STATUS_MODULE_DETACHED;

    pthread_mutex_lock(&g_recv_mutex);
    while (g_recv_count == 0) {
        pthread_cond_wait(&g_recv_cond, &g_recv_mutex);
    }
    recv_slot_t *slot = &g_recv_queue[g_recv_head];
    if (srcmac) *srcmac = slot->srcmac;
    memcpy(data, slot->data, slot->len);
    *len = slot->len;
    g_recv_head = (g_recv_head + 1) % RECV_QUEUE_DEPTH;
    g_recv_count--;
    pthread_mutex_unlock(&g_recv_mutex);
    return STATUS_SUCCESS;
}

// Bounded-wait variant of recv_wifi_packet.
// Not part of drivers.h — declared extern where needed (see test_wifi.c).
// Returns STATUS_SUCCESS if a packet was dequeued, STATUS_MODULE_DETACHED
// if the driver isn't initialized, or -1 on timeout. On timeout the
// out parameters are left untouched.
Status recv_wifi_packet_timeout(uint64_t *srcmac,
                                uint8_t data[MAX_WIFI_RECV_PACKET_LEN],
                                BufLen *len,
                                uint32_t timeout_ms) {
    if (!g_initialized) return STATUS_MODULE_DETACHED;

    uint64_t deadline = now_ms_mono() + timeout_ms;
    pthread_mutex_lock(&g_recv_mutex);
    while (g_recv_count == 0) {
        uint64_t now = now_ms_mono();
        if (now >= deadline) {
            pthread_mutex_unlock(&g_recv_mutex);
            return (Status)-1;
        }
        uint64_t rem = deadline - now;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec  += rem / 1000;
        ts.tv_nsec += (rem % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        int r = pthread_cond_timedwait(&g_recv_cond, &g_recv_mutex, &ts);
        if (r == ETIMEDOUT) {
            pthread_mutex_unlock(&g_recv_mutex);
            return (Status)-1;
        }
        // Spurious wakeup or signalled — loop checks g_recv_count again.
    }
    recv_slot_t *slot = &g_recv_queue[g_recv_head];
    if (srcmac) *srcmac = slot->srcmac;
    memcpy(data, slot->data, slot->len);
    *len = slot->len;
    g_recv_head = (g_recv_head + 1) % RECV_QUEUE_DEPTH;
    g_recv_count--;
    pthread_mutex_unlock(&g_recv_mutex);
    return STATUS_SUCCESS;
}

// Drain (discard) any queued packets. Used by ATP05 to avoid an old echo
// from a previous cycle being mistaken for the current cycle's reply.
void wifi_drain_recv_queue(void) {
    pthread_mutex_lock(&g_recv_mutex);
    g_recv_head = g_recv_tail = g_recv_count = 0;
    pthread_mutex_unlock(&g_recv_mutex);
}

// Optional: query the ESP32's link status (not in drivers.h, but useful for tests)
typedef struct {
    bool connected;
    int  rssi_dbm;
    uint8_t self_mac[6];
} wifi_link_status_t;

Status wifi_query_status(wifi_link_status_t *out) {
    if (!g_initialized) return STATUS_MODULE_DETACHED;
    if (send_frame(WIFI_MSG_STATUS_Q, NULL, 0) != 0) return -1;
    uint8_t pl[WIFI_MAX_PAYLOAD]; uint16_t pl_len = 0;
    uint8_t resp = wait_response(2000, pl, &pl_len);
    if (resp != WIFI_MSG_STATUS || pl_len < 10) return -1;
    out->connected = pl[0] != 0;
    int16_t rssi = (int16_t)(((uint16_t)pl[2] << 8) | pl[3]);
    out->rssi_dbm = rssi;
    memcpy(out->self_mac, &pl[4], 6);
    return STATUS_SUCCESS;
}