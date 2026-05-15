// test_lora.c
//
// Standalone test harness for the LoRa driver. Supports:
//
//   test_lora send <device>           — simple TX loop (every 2 sec)
//   test_lora recv <device>           — simple RX loop (prints to stdout)
//   test_lora atp03-tx <device>       — ATP03 ping mode: 1 packet every 10 sec
//                                       for 5 minutes (30 packets total)
//   test_lora atp03-rx <device> <csv> — ATP03 log mode: listen for the full
//                                       5 minutes, log every packet's
//                                       timestamp, RSSI, SNR to <csv>
//
// Example (range test, two laptops):
//   Buoy side  (UCT upper campus):   ./test_lora atp03-tx /dev/cu.usbserial-0001
//   Ship side  (Rondebosch Common):  ./test_lora atp03-rx /dev/cu.usbserial-5  atp03_1km.csv
//
// The ATP03 modes are self-terminating after the test duration so you don't
// have to remember to stop them.

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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "../include/drivers.h"

// Exposed by lora.c for path-loss measurement campaigns.
extern int g_last_rssi_dbm;
extern int g_last_snr_db;

// ATP03 parameters (per the procedure spec).
#define ATP03_INTERVAL_S     10
#define ATP03_DURATION_S     300   // 5 minutes
#define ATP03_TOTAL_PACKETS  (ATP03_DURATION_S / ATP03_INTERVAL_S)  // = 30

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

static void timestamp_iso8601(char *buf, size_t buflen) {
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(buf, buflen, "%Y-%m-%dT%H:%M:%S", &tm);
}

static void print_bytes(const uint8_t *data, BufLen len) {
    for (BufLen i = 0; i < len; i++) {
        printf("%02X", data[i]);
    }
}

// ----------------------------------------------------------------------------
// Simple send / recv loops (for desk testing)
// ----------------------------------------------------------------------------

static int do_send(void) {
    printf("[test] initializing LoRa module...\n");
    if (initialize_lora_module() != STATUS_SUCCESS) {
        fprintf(stderr, "[test] init failed\n");
        return 1;
    }
    printf("[test] init OK. Byterate ~%.1f bytes/sec\n", get_lora_byterate());

    uint32_t counter = 0;
    while (1) {
        uint8_t packet[16];
        snprintf((char *)packet, sizeof(packet), "PKT%05u", counter++);
        BufLen len = (BufLen)strlen((char *)packet);

        printf("[test] sending: ");
        print_bytes(packet, len);
        printf(" (\"%s\")\n", (char *)packet);
        fflush(stdout);

        Status s = send_lora_packet(packet, len);
        if (s != STATUS_SUCCESS) {
            fprintf(stderr, "[test] send failed (status=%d)\n", s);
        } else {
            printf("[test] send OK\n");
        }

        sleep(2);
    }
    return 0;
}

static int do_recv(void) {
    printf("[test] initializing LoRa module...\n");
    if (initialize_lora_module() != STATUS_SUCCESS) {
        fprintf(stderr, "[test] init failed\n");
        return 1;
    }
    printf("[test] init OK. Listening...\n");

    while (1) {
        uint8_t data[MAX_LORA_RECV_PACKET_LEN];
        BufLen len = 0;
        Status s = recv_lora_packet(data, &len, 30000);
        if (s == STATUS_RECEIVE_TIMEOUT) {
            printf("[test] (timeout, still listening)\n");
            continue;
        }
        if (s != STATUS_SUCCESS) {
            fprintf(stderr, "[test] recv failed (status=%d)\n", s);
            continue;
        }
        printf("[test] received %u bytes: ", (unsigned)len);
        print_bytes(data, len);
        printf("  (\"%.*s\")  rssi=%d  snr=%d\n",
               (int)len, (char *)data, g_last_rssi_dbm, g_last_snr_db);
        fflush(stdout);
    }
    return 0;
}

// ----------------------------------------------------------------------------
// ATP03: range test
// ----------------------------------------------------------------------------

static int do_atp03_tx(void) {
    printf("[atp03-tx] initializing LoRa module...\n");
    if (initialize_lora_module() != STATUS_SUCCESS) {
        fprintf(stderr, "[atp03-tx] init failed\n");
        return 1;
    }
    printf("[atp03-tx] init OK. Sending %d packets, %d sec apart.\n",
           ATP03_TOTAL_PACKETS, ATP03_INTERVAL_S);
    printf("[atp03-tx] total test duration: %d sec (%d min)\n",
           ATP03_DURATION_S, ATP03_DURATION_S / 60);

    for (int i = 0; i < ATP03_TOTAL_PACKETS; i++) {
        uint8_t packet[16];
        snprintf((char *)packet, sizeof(packet), "PING%03d", i);
        BufLen len = (BufLen)strlen((char *)packet);

        char ts[32];
        timestamp_iso8601(ts, sizeof(ts));

        Status s = send_lora_packet(packet, len);
        printf("[%s] tx #%d/%d: %s\n",
               ts, i + 1, ATP03_TOTAL_PACKETS,
               (s == STATUS_SUCCESS) ? "OK" : "FAIL");
        fflush(stdout);

        // Sleep until the next interval boundary. Skip on last packet.
        if (i + 1 < ATP03_TOTAL_PACKETS) {
            sleep(ATP03_INTERVAL_S);
        }
    }

    printf("[atp03-tx] test complete. Sent %d packets.\n", ATP03_TOTAL_PACKETS);
    shutdown_lora_module();
    return 0;
}

static int do_atp03_rx(const char *csv_path) {
    FILE *csv = fopen(csv_path, "w");
    if (csv == NULL) {
        fprintf(stderr, "[atp03-rx] cannot open %s for writing\n", csv_path);
        return 1;
    }
    fprintf(csv, "timestamp,packet_num,payload,rssi_dbm,snr_db,len_bytes\n");
    fflush(csv);

    printf("[atp03-rx] initializing LoRa module...\n");
    if (initialize_lora_module() != STATUS_SUCCESS) {
        fprintf(stderr, "[atp03-rx] init failed\n");
        fclose(csv);
        return 1;
    }
    printf("[atp03-rx] init OK. Listening for %d sec, logging to %s.\n",
           ATP03_DURATION_S, csv_path);
    printf("[atp03-rx] (start the TX side now if you haven't yet)\n");

    time_t start = time(NULL);
    time_t deadline = start + ATP03_DURATION_S;
    int packets_received = 0;

    while (time(NULL) < deadline) {
        uint8_t data[MAX_LORA_RECV_PACKET_LEN];
        BufLen len = 0;

        // Compute remaining time and use that as the recv timeout.
        time_t remaining = deadline - time(NULL);
        if (remaining <= 0) break;
        uint32_t timeout_ms = (uint32_t)(remaining * 1000);
        if (timeout_ms > 30000) timeout_ms = 30000;  // cap for responsiveness

        Status s = recv_lora_packet(data, &len, timeout_ms);
        if (s == STATUS_RECEIVE_TIMEOUT) {
            // Just keep listening — nothing arrived this round.
            continue;
        }
        if (s != STATUS_SUCCESS) {
            fprintf(stderr, "[atp03-rx] recv error (status=%d)\n", s);
            continue;
        }

        // Got a packet. Log it.
        packets_received++;
        char ts[32];
        timestamp_iso8601(ts, sizeof(ts));

        // Make payload safe for CSV (it's the firmware counter, so just digits;
        // but be defensive).
        char payload[64];
        size_t pcopy = (len < sizeof(payload) - 1) ? len : sizeof(payload) - 1;
        memcpy(payload, data, pcopy);
        payload[pcopy] = '\0';

        fprintf(csv, "%s,%d,%s,%d,%d,%u\n",
                ts, packets_received, payload,
                g_last_rssi_dbm, g_last_snr_db, (unsigned)len);
        fflush(csv);

        printf("[%s] rx #%d  payload=\"%s\"  rssi=%d dBm  snr=%d dB\n",
               ts, packets_received, payload,
               g_last_rssi_dbm, g_last_snr_db);
        fflush(stdout);
    }

    int total_expected = ATP03_TOTAL_PACKETS;
    double pct = (100.0 * packets_received) / total_expected;
    printf("[atp03-rx] test complete.\n");
    printf("[atp03-rx] received %d of %d expected packets (%.1f%%)\n",
           packets_received, total_expected, pct);
    printf("[atp03-rx] PASS criterion: >= 90%%  ->  %s\n",
           (pct >= 90.0) ? "PASS" : "FAIL");

    fclose(csv);
    shutdown_lora_module();
    return 0;
}

// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------

static void usage(const char *argv0) {
    fprintf(stderr,
        "usage:\n"
        "  %s send       <device>            - simple TX loop (every 2 s)\n"
        "  %s recv       <device>            - simple RX loop\n"
        "  %s atp03-tx   <device>            - ATP03 ping: 30 packets, 10 s apart\n"
        "  %s atp03-rx   <device> <csvfile>  - ATP03 log: 5 min, RSSI/SNR -> CSV\n"
        "\n"
        "device defaults to LORA_SERIAL_DEVICE env or /dev/cu.usbserial-0001\n",
        argv0, argv0, argv0, argv0);
}

int main(int argc, char **argv) {
    if (argc < 2) { usage(argv[0]); return 1; }

    const char *mode = argv[1];

    if (strcmp(mode, "send") == 0 || strcmp(mode, "recv") == 0) {
        if (argc >= 3) setenv("LORA_SERIAL_DEVICE", argv[2], 1);
        return (strcmp(mode, "send") == 0) ? do_send() : do_recv();
    }

    if (strcmp(mode, "atp03-tx") == 0) {
        if (argc < 3) { usage(argv[0]); return 1; }
        setenv("LORA_SERIAL_DEVICE", argv[2], 1);
        return do_atp03_tx();
    }

    if (strcmp(mode, "atp03-rx") == 0) {
        if (argc < 4) { usage(argv[0]); return 1; }
        setenv("LORA_SERIAL_DEVICE", argv[2], 1);
        return do_atp03_rx(argv[3]);
    }

    fprintf(stderr, "unknown mode: %s\n", mode);
    usage(argv[0]);
    return 1;
}
