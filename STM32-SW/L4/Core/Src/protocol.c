// protocol.c
//
// Application/transport/network layer for LoRa + WiFi communication.
// Sits between netio.h (link layer) and control.h (STM32/storage layer).

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../Inc/netio.h"
#include "../Inc/rust_wifi.h"
#include "../Inc/control.h"

// ---------------------------------------------------------------------------
// Compile-time configuration
// ---------------------------------------------------------------------------

// LoRa device address for this module.
#ifndef MODULE_LORA_ADDR
#  define MODULE_LORA_ADDR ((uint16_t)0x0001u)
#endif

// ---------------------------------------------------------------------------
// LoRa packet wire format
//
//  Offset  Size  Field
//  0       2     src_addr_and_seq  — bits [14:0] = controller LoRa address,
//                                     bit  [15]   = sequence bit (ping-pong)
//  2       2     dst_addr          — module LoRa address (us)
//  4       1     cmd_count         — number of commands that follow
//  5       ?     commands          — packed LoRaCmd stream (see app.rs)
//  last-1  2     crc16             — CRC-16 over bytes [0 .. len-2]
//
// Reply format (module → controller):
//  0       2     dst_addr_and_seq  — controller address | (seq << 15)
//  2       2     src_addr          — module LoRa address
//  4       ?     LoRaModuleState   — serialised (see app.rs for layout)
//  last-1  2     crc16             — CRC-16 over bytes [0 .. len-2]
// ---------------------------------------------------------------------------

#define LORA_HDR_LEN 4u
#define LORA_CRC_LEN 2u
#define LORA_MIN_LEN (LORA_HDR_LEN + LORA_CRC_LEN)

// LoRaModuleState serialised size (from app.rs):
//   status_flags(1) + storage_policy(1) + StorageInfo(4+8+4+4) + LoRaRecvWindow(2+2) + GpsInfo(4+4)
//   = 1+1+20+4+8 = 34 bytes
#define LORA_STATE_SERIALISED_LEN  34u

// Total reply packet length:
//   src(2) + dst_and_seq(2) + state(30) + crc(2) = 36 bytes
#define LORA_REPLY_LEN (2u + 2u + LORA_STATE_SERIALISED_LEN + LORA_CRC_LEN)

_Static_assert(LORA_REPLY_LEN <= MAX_LORA_SEND_PACKET_LEN,
    "LoRa reply exceeds MAX_LORA_SEND_PACKET_LEN");


// ---------------------------------------------------------------------------
// LoRa command discriminators (mirrors app.rs LoRaCmd constants)
// ---------------------------------------------------------------------------

#define CMD_ENABLE_WIFI         1u
#define CMD_DISABLE_WIFI        2u
#define CMD_START_DATA_DUMP     5u
#define CMD_CANCEL_DATA_DUMP    6u
#define CMD_SET_LORA_RECV_WINDOW 7u
#define CMD_SET_OVERWRITABLE    8u
#define CMD_SET_STORAGE_POLICY  9u

#define STORAGE_POLICY_OVERWRITE 1u
#define STORAGE_POLICY_PRESERVE  2u

// Status flags (mirrors app.rs LoRaModuleState::STATUS_*)
#define STATUS_WIFI_ON      (1u << 0)
#define STATUS_WIFI_DUMPING (1u << 3)


// ---------------------------------------------------------------------------
// Little-endian read/write helpers
// ---------------------------------------------------------------------------

static inline uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] <<  8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static inline uint64_t read_u64_le(const uint8_t *p)
{
    return (uint64_t)read_u32_le(p)
         | ((uint64_t)read_u32_le(p + 4) << 32);
}

static inline void write_u8(uint8_t *p, uint8_t v)   { *p = v; }

static inline void write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}

static inline void write_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >>  8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void write_u64_le(uint8_t *p, uint64_t v)
{
    write_u32_le(p,     (uint32_t)(v));
    write_u32_le(p + 4, (uint32_t)(v >> 32));
}


// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  status_flags;
    uint8_t  storage_policy;           // STORAGE_POLICY_* values
    uint16_t lora_recv_window_on;      // seconds
    uint16_t lora_recv_window_total;   // seconds
} ModuleState;

static ModuleState g_state = {
    .status_flags            = 0,
    .storage_policy          = STORAGE_POLICY_PRESERVE,
    .lora_recv_window_on     = 5,
    .lora_recv_window_total  = 5,
};


// ---------------------------------------------------------------------------
// WiFi data-dump state
// ---------------------------------------------------------------------------

static bool g_dump_active = false;
static uint32_t g_dump_next_block = 0;   // next block index to enqueue
static uint32_t g_dump_last_block = 0;   // inclusive upper bound

// Scratch buffer for reading blocks before pushing into Rust WiFi layer.
static uint8_t g_block_buf[STORAGE_BLOCK_SIZE + 8];


// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

static void lora_send_reply(uint16_t controller_addr_and_seq_bit);
static uint16_t serialise_module_state(uint8_t *buf);
static void process_lora_command(const uint8_t *cmd_buf, BufLen cmd_len, uint16_t controller_addr);
static void apply_cmd_enable_wifi(uint16_t controller_addr);
static void apply_cmd_disable_wifi(void);
static void apply_cmd_start_data_dump(const uint8_t *p, uint16_t controller_addr);
static void apply_cmd_cancel_data_dump(void);
static void apply_cmd_set_lora_recv_window(const uint8_t *p);
static void apply_cmd_set_overwritable(const uint8_t *p);
static void apply_cmd_set_storage_policy(const uint8_t *p);
static void wifi_timer_set(uint32_t n);
static void wifi_timeout_handler(void);
static void wifi_ping_arm(void);
static bool fill_wifi_buffer(void);
static void wifi_do_send_next(void);
static void get_gps_info(uint32_t *lat, uint32_t *lon);


// ---------------------------------------------------------------------------
// LoRa receive path
//
// Called by Tamryn's driver when a packet has been received.
// We parse, validate, execute commands, then send a reply.
// ---------------------------------------------------------------------------

void recv_lora_packet(uint8_t *data, BufLen len)
{
    // Minimum: header (5) + at least 0 commands + CRC (2).
    if (len < LORA_MIN_LEN) {
        return;
    }

    // Parse header
    uint16_t controller_addr_and_seq = read_u16_le(&data[0]);
    uint16_t module_addr = read_u16_le(&data[2]);

    uint16_t controller_addr = controller_addr_and_seq & 0x7FFFu;

    // Drop packets not addressed to us or broadcast.
    if (module_addr != MODULE_LORA_ADDR && module_addr != 0x7fff) {
        return;
    }

    // --- CRC check ---
    uint16_t rx_crc = read_u16_le(&data[len - 2]);
    uint16_t calc_crc = crc16(data, (uint32_t)(len - 2));
    if (rx_crc != calc_crc) {
        // Silently drop corrupt packets; the controller will retry.
        char printbuf[100];
        snprintf(printbuf, 100, "shaun says bad crc: packet %d calculated %d\n", rx_crc, calc_crc);
        shaun_debug(printbuf);
        return;
    }

    // --- Parse and execute commands ---
    // Commands start at offset 5 and end before the 2-byte CRC.
    BufLen cmd_payload_len = (BufLen)(len - LORA_HDR_LEN - LORA_CRC_LEN);
    process_lora_command(&data[LORA_HDR_LEN], cmd_payload_len, controller_addr);

    // --- Send reply ---
    lora_send_reply(controller_addr_and_seq);
}


// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

static void process_lora_command(const uint8_t *buf, BufLen len, uint16_t controller_addr)
{
    if (g_dump_active && g_dump_last_block < g_dump_next_block && wifi_sent_all()) {
        g_dump_active = false;
        g_state.status_flags &= (uint8_t)~STATUS_WIFI_DUMPING;
        cancel_timeout();
    }

    BufLen pos = 0;

    while (pos < len) {
        uint8_t discriminator = buf[pos++];

        switch (discriminator) {
            case CMD_ENABLE_WIFI:
                apply_cmd_enable_wifi(controller_addr);
                break;

            case CMD_DISABLE_WIFI:
                apply_cmd_disable_wifi();
                break;

            case CMD_START_DATA_DUMP:
                if (pos + 16 > len) return;  // need 8+8 bytes
                apply_cmd_start_data_dump(&buf[pos], controller_addr);
                pos += 16;
                break;

            case CMD_CANCEL_DATA_DUMP:
                apply_cmd_cancel_data_dump();
                break;

            case CMD_SET_LORA_RECV_WINDOW:
                if (pos + 4 > len) return;   // need 2+2 bytes
                apply_cmd_set_lora_recv_window(&buf[pos]);
                pos += 4;
                break;

            case CMD_SET_OVERWRITABLE:
                if (pos + 8 > len) return;   // need u64
                apply_cmd_set_overwritable(&buf[pos]);
                pos += 8;
                break;

            case CMD_SET_STORAGE_POLICY:
                if (pos + 1 > len) return;   // need u8
                apply_cmd_set_storage_policy(&buf[pos]);
                pos += 1;
                break;

            default:
                // Unknown discriminator — stop parsing rather than skipping
                // an unknown number of bytes.
                return;
        }
    }
}


// ---------------------------------------------------------------------------
// Individual command handlers
// ---------------------------------------------------------------------------

static void apply_cmd_enable_wifi(uint16_t controller_addr)
{
    if (g_state.status_flags & STATUS_WIFI_ON) {
        return;  // idempotent
    }
    Status s = power_up_wifi();
    if (s == STATUS_SUCCESS) {
        g_state.status_flags |= STATUS_WIFI_ON;

        wifi_ping_arm();
    }
}

static void apply_cmd_disable_wifi(void)
{
    if (!(g_state.status_flags & STATUS_WIFI_ON)) {
        return;  // idempotent
    }
    apply_cmd_cancel_data_dump();
    cancel_timeout_wifi_ping();
    power_down_wifi();
    g_state.status_flags &= (uint8_t)~STATUS_WIFI_ON;
}

static void apply_cmd_start_data_dump(const uint8_t *p, uint16_t controller_addr)
{
    uint64_t from_block = read_u64_le(p);
    uint64_t to_block = read_u64_le(p + 8);

    // Ensure WiFi is up before starting a dump.
    if (!(g_state.status_flags & STATUS_WIFI_ON)) {
        return;
    }

    // Clamp range to what's actually readable.
    uint32_t first_readable = storage_first_readable_block();
    uint32_t last_readable  = storage_last_readable_block();

    uint32_t from = (uint32_t)from_block;
    uint32_t to = (uint32_t)to_block;

    if (from < first_readable) from = first_readable;
    if (to > last_readable) to = last_readable;

    if (from > to) {
        return; // Empty range after clamping.
    }

    // Make sure any buffered writes are committed before we start reading.
    flush_block_buffer_to_disk();

    g_dump_active = true;
    g_dump_next_block = from;
    g_dump_last_block = to;
    g_state.status_flags |= STATUS_WIFI_DUMPING;

    // Attempt to connect. wifi_connect returns true if wifi_send_next
    // should be called immediately (i.e. there's already data queued).
    bool send_now2 = wifi_connect(
        controller_addr,
        wifi_timer_set,
        cancel_timeout,
        get_time_since_epoch_ms,
        crc16
    );
    if (send_now2) {
        wifi_do_send_next();
    }
}

static void apply_cmd_cancel_data_dump(void)
{
    g_dump_active = false;
    g_state.status_flags &= (uint8_t)~STATUS_WIFI_DUMPING;
}

static void apply_cmd_set_lora_recv_window(const uint8_t *p)
{
    uint16_t on_period    = read_u16_le(p);
    uint16_t total_period = read_u16_le(p + 2);

    g_state.lora_recv_window_on = on_period;
    g_state.lora_recv_window_total = total_period;

    set_lora_recv_window(on_period, total_period);
}

static void apply_cmd_set_overwritable(const uint8_t *p)
{
    uint64_t up_to = read_u64_le(p);

    // allow_overwrite takes a block index that is <= storage_last_readable_block.
    uint32_t last = storage_last_readable_block();
    if (up_to > last) {
        up_to = last;
    }

    allow_overwrite(up_to);
}

static void apply_cmd_set_storage_policy(const uint8_t *p)
{
    uint8_t policy = *p;
    switch (policy) {
        case STORAGE_POLICY_OVERWRITE:
            g_state.storage_policy = STORAGE_POLICY_OVERWRITE;
            set_overwrite_policy(STORAGE_POLICY_OVERWRITE);
            break;
        case STORAGE_POLICY_PRESERVE:
            g_state.storage_policy = STORAGE_POLICY_PRESERVE;
            set_overwrite_policy(STORAGE_POLICY_PRESERVE);
            break;
        default:
            break;  // Unknown policy — ignore.
    }
}


// ---------------------------------------------------------------------------
// LoRa reply serialisation
//
// Reply wire format:
//  [0:1]   module LoRa address (u16 LE)
//  [2:3]   controller address | (seq_bit << 15) (u16 LE)
//  [4:33]  LoRaModuleState (30 bytes)
//  [34:35] CRC-16 over bytes [0:33]
//
// LoRaModuleState layout (mirrors app.rs):
//   status_flags    : u8       (1)
//   storage_policy  : u8       (1)
//   StorageInfo:
//     total_blocks  : u32 LE   (4)
//     avail_begin   : u64 LE   (8)
//     avail_end_off : u32 LE   (4)  = available_end - available_begin
//     overwr_off    : u32 LE   (4)  = overwritable_end - available_begin
//   LoRaRecvWindow:
//     on_period     : u16 LE   (2)
//     total_period  : u16 LE   (2)
//   GpsInfo:
//     lat           : u32 LE   (4)
//     lon           : u32 LE   (4)
//
// Total state: 1+1+4+8+4+4+2+2+4+4 = 34 bytes
//
// Hmm — this is 34, not 30. Let me recount and correct the constant above.
//   1+1+4+8+4+4+2+2+4+4 = 34.
// LORA_STATE_SERIALISED_LEN should be 34, LORA_REPLY_LEN = 2+2+34+2 = 40.
// Correcting below; the _Static_assert will catch any future drift.
// ---------------------------------------------------------------------------


#undef  LORA_REPLY_LEN
#define LORA_REPLY_LEN (2u + 2u + LORA_STATE_SERIALISED_LEN + LORA_CRC_LEN)

_Static_assert(LORA_REPLY_LEN <= MAX_LORA_SEND_PACKET_LEN, "LoRa reply exceeds MAX_LORA_SEND_PACKET_LEN");

static uint8_t g_lora_reply_buf[LORA_REPLY_LEN];

static uint16_t serialise_module_state(uint8_t *buf)
{
    uint16_t pos = 0;

    // status_flags
    write_u8(buf + pos, g_state.status_flags);
    pos += 1;

    // storage_policy
    write_u8(buf + pos, g_state.storage_policy);
    pos += 1;

    // StorageInfo
    uint32_t total = storage_total_blocks();
    uint64_t avail_b = storage_first_readable_block();
    uint64_t avail_e = storage_last_readable_block();
    uint64_t prot_end = storage_first_protected_block();

    // overwritable_end = first_protected_block (everything before it may be
    // overwritten).  The Rust side stores this relative to available_begin.
    uint32_t avail_len  = (avail_e >= avail_b) ? (avail_e - avail_b) : 0u;
    uint32_t overwr_off = (prot_end >= avail_b) ? (prot_end - avail_b) : 0u;

    write_u32_le(buf + pos, total); pos += 4;
    write_u64_le(buf + pos, (uint64_t)avail_b);  pos += 8;
    write_u32_le(buf + pos, avail_len); pos += 4;
    write_u32_le(buf + pos, overwr_off); pos += 4;

    // LoRaRecvWindow
    write_u16_le(buf + pos, g_state.lora_recv_window_on); pos += 2;
    write_u16_le(buf + pos, g_state.lora_recv_window_total); pos += 2;

    // GpsInfo — Glen needs to provide get_gps_info().
    uint32_t lat = 0, lon = 0;
    get_gps_info(&lat, &lon);
    write_u32_le(buf + pos, lat); pos += 4;
    write_u32_le(buf + pos, lon); pos += 4;

    return pos;  // Should equal LORA_STATE_SERIALISED_LEN.
}

static void lora_send_reply(uint16_t controller_addr_and_seq_bit)
{
    uint8_t *buf = g_lora_reply_buf;
    uint16_t pos = 0;

    // Header
    write_u16_le(buf + pos, controller_addr_and_seq_bit);
    pos += 2;
    write_u16_le(buf + pos, MODULE_LORA_ADDR);
    pos += 2;

    // State payload
    pos += serialise_module_state(buf + pos);

    // CRC over everything so far
    uint16_t crc = crc16(buf, pos); // TODO
    write_u16_le(buf + pos, crc);
    pos += 2;

    send_lora_packet(buf, (BufLen)pos);
}


// ---------------------------------------------------------------------------
// WiFi timer callbacks
// ---------------------------------------------------------------------------

static void wifi_timer_set(uint32_t n)
{
    call_after_n_ms(n, wifi_timeout_handler);
}

static void wifi_timeout_handler()
{
    bool send_next = wifi_on_timeout();
    if (send_next) {
        wifi_do_send_next();
    }
}

#define MAC_BCAST ((uint64_t)0xffffffffffff)

static uint8_t WIFI_PING_BUF[WIFI_PING_SIZE];

static void wifi_ping_handler()
{
    // if (!g_dump_active) {
        wifi_ping_write(MODULE_LORA_ADDR, crc16, WIFI_PING_BUF);
        send_wifi_packet(MAC_BCAST, WIFI_PING_BUF, WIFI_PING_SIZE);
    // }
}

static void wifi_ping_arm(void)
{
    call_repeatedly_after_n_ms_wifi_ping(1000, wifi_ping_handler);
}

// ---------------------------------------------------------------------------
// WiFi send pump
// ---------------------------------------------------------------------------

static void wifi_do_send_next(void)
{
    uint64_t mac;
    const uint8_t *payload;
    uint16_t payload_len;

    while (wifi_send_next(&mac, &payload, &payload_len)) {
        send_wifi_packet(mac, (uint8_t *)payload, payload_len);
    }
}


// ---------------------------------------------------------------------------
// WiFi receive path
//
// Called by Tamryn's driver when a packet arrives on the WiFi interface.
// We hand it to the Rust reliability layer, which may produce more sends
// and/or advance the dump state.
// ---------------------------------------------------------------------------

void recv_wifi_packet(uint64_t *macsrc, uint8_t *data, uint16_t len)
{
    bool send_next = wifi_on_recv_ack(*macsrc, data, len);
    if (send_next) {
        wifi_do_send_next();
    }

    // If the Rust layer has consumed some of its buffer, try to top it up.
    if (g_dump_active) {
        bool also_send = fill_wifi_buffer();
        if (also_send) {
            wifi_do_send_next();
        }
    }
}


// ---------------------------------------------------------------------------
// Block-reading pump
//
// Reads blocks from storage into the Rust WiFi send buffer until either
// we run out of blocks to dump or the buffer is too full to fit another
// block.  Returns true if wifi_send_next should be called.
// ---------------------------------------------------------------------------

static bool fill_wifi_buffer(void)
{
    if (!g_dump_active) {
        return false;
    }

    bool ready_to_send = true;

    while (g_dump_next_block <= g_dump_last_block
           && wifi_available_payload_bytes() >= STORAGE_BLOCK_SIZE)
    {
        write_u64_le(g_block_buf, g_dump_next_block);
        uint32_t bytes_read = read_block(g_dump_next_block, g_block_buf + 8);

        bool rts = wifi_push_message(g_block_buf, (uint16_t)(bytes_read + 8));
        ready_to_send |= rts;
        g_dump_next_block++;
    }

    return ready_to_send;
}


void get_gps_info(uint32_t *lat, uint32_t *lon) {
    *lat = 0;
    *lon = 0;
}
