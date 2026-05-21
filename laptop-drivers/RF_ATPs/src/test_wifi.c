// test_wifi.c
//
// Standalone test harness for the WiFi driver. Implements ATP02
// (throughput) and ATP05 (sleep/wake) plus lower-level modes for
// bench debugging.
//
// Usage examples:
//
//   # ATP02 ship side (receive 100 MB, report throughput)
//   WIFI_ROLE=ship WIFI_SERIAL_DEVICE=/dev/cu.usbserial-0001 \
//     ./test_wifi --atp 02 --side ship
//
//   # ATP02 buoy side (send 100 MB)
//   WIFI_ROLE=buoy WIFI_SERIAL_DEVICE=/dev/cu.usbmodem2101 \
//     ./test_wifi --atp 02 --side buoy
//
//   # ATP05 buoy side (sleep/wake 3 cycles)
//   WIFI_ROLE=buoy ./test_wifi --atp 05 --side buoy --cycles 3
//
//   # Simple bidirectional echo
//   WIFI_ROLE=ship ./test_wifi --mode echo
//   WIFI_ROLE=buoy ./test_wifi --mode send --bytes 10000
//
// Notes:
//   - The driver picks the device via WIFI_SERIAL_DEVICE env var.
//   - The ESP32's role (ship/buoy firmware) must match WIFI_ROLE.

#if defined(__linux__)
  #ifndef _DEFAULT_SOURCE
  #define _DEFAULT_SOURCE
  #endif
  #ifndef _POSIX_C_SOURCE
  #define _POSIX_C_SOURCE 200809L
  #endif
#endif

#include "../include/drivers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Forward declaration of the optional status function from wifi.c
typedef struct {
    bool connected;
    int  rssi_dbm;
    uint8_t self_mac[6];
} wifi_link_status_t;
extern Status wifi_query_status(wifi_link_status_t *out);

// Bounded-wait variant of recv_wifi_packet, implemented in wifi.c but
// not part of drivers.h. Returns STATUS_SUCCESS, STATUS_MODULE_DETACHED,
// or -1 on timeout.
extern Status recv_wifi_packet_timeout(uint64_t *srcmac,
                                       uint8_t data[MAX_WIFI_RECV_PACKET_LEN],
                                       BufLen *len,
                                       uint32_t timeout_ms);

// Discard any queued received packets. Used to prevent a stale echo
// from a previous ATP05 cycle being matched against the current cycle.
extern void wifi_drain_recv_queue(void);

// Fire-and-forget send (no per-frame ACK wait). Used by run_send to
// saturate the UART instead of being rate-limited by the round-trip
// to the ESP32. Reliability is delegated to TCP.
extern Status send_wifi_packet_nowait(uint64_t destmac, const uint8_t *data, BufLen len);

// Returns the cumulative count of asynchronous NACKs received since the
// last initialize_wifi_module(). Used to surface failures that the
// fire-and-forget path can't see directly.
extern uint64_t wifi_get_nack_count(void);

// Cumulative count of MSG_RECV frames seen by the reader thread.
// Distinguishes "ESP32 not forwarding" from "test code draining too slowly."
extern uint64_t wifi_get_recv_frame_count(void);
extern uint64_t wifi_get_recv_bytes_count(void);

// Block until the ESP32 has drained its UART command queue (proven by a
// successful status round-trip). Call at the end of a no-wait send run
// so throughput math reflects delivered bytes, not queued bytes.
extern Status wifi_drain_pending(uint32_t timeout_ms);

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

static void print_mac(const uint8_t *m) {
    printf("%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}

// Format a wall-clock timestamp (CLOCK_REALTIME) as "YYYY-MM-DD HH:MM:SS"
// into the given buffer. Used for stamping output files.
static void format_timestamp(char *out, size_t outlen) {
    time_t t = time(NULL);
    struct tm tm_local;
    localtime_r(&t, &tm_local);
    strftime(out, outlen, "%Y-%m-%d %H:%M:%S", &tm_local);
}

// ---------------- Modes ---------------- //

static int run_send(uint64_t total_bytes, uint16_t pkt_size, uint64_t destmac,
                    uint32_t pace_us, const char *output_path) {
    if (pkt_size == 0 || pkt_size > MAX_WIFI_SEND_PACKET_LEN) pkt_size = MAX_WIFI_SEND_PACKET_LEN;
    fprintf(stderr, "[test] sending %llu bytes in %u-byte packets (fire-and-forget, pace=%uus)\n",
            (unsigned long long)total_bytes, pkt_size, pace_us);

    uint8_t *buf = malloc(pkt_size);
    if (!buf) return -1;
    for (uint16_t i = 0; i < pkt_size; i++) buf[i] = (uint8_t)(i & 0xFF);

    // Baseline NACK count so we can report only NACKs that occurred
    // during this send, not ones inherited from earlier calls.
    uint64_t nacks_at_start = wifi_get_nack_count();

    uint64_t sent_bytes = 0;
    uint64_t sent_packets = 0;
    uint64_t failed_packets = 0;     // UART-level write failures (rare)
    uint64_t start = now_ms();
    uint64_t last_report = start;

    // RSSI sampling. Query the buoy's STA-side RSSI once per second
    // (same cadence as the progress log). The status round-trip costs
    // ~5-10ms via UART but the buoy isn't transmitting during that time
    // anyway. Min/max bracket the range; mean is the headline figure
    // for the report.
    int rssi_sum = 0;
    int rssi_min = 0;
    int rssi_max = 0;
    int rssi_samples = 0;
    bool rssi_first = true;

    while (sent_bytes < total_bytes) {
        uint64_t remaining = total_bytes - sent_bytes;
        uint16_t this_len = pkt_size;
        if (remaining < this_len) this_len = (uint16_t)remaining;

        // Embed sequence number in first 8 bytes (helps integrity check on rx)
        memcpy(buf, &sent_packets, sizeof(sent_packets));

        Status s = send_wifi_packet_nowait(destmac, buf, this_len);
        if (s == STATUS_SUCCESS) {
            sent_bytes += this_len;
            sent_packets++;
        } else {
            failed_packets++;
            // A UART-level write failure usually means the device went
            // away; backing off briefly is the most we can do.
            if (failed_packets > 100 && failed_packets > sent_packets / 4) {
                fprintf(stderr, "[test] too many UART write failures, aborting\n");
                break;
            }
            usleep(10000);
        }

        // Inter-frame pacing: prevents the laptop's kernel UART buffer
        // from running ahead of the wire. At 921600 baud, a 519-byte
        // frame physically takes ~5.6ms to transmit; without this sleep
        // the kernel buffers many frames ahead and the ESP32's UART RX
        // buffer overflows when our processing-per-frame exceeds line
        // rate. pace_us = 1000-2000 gives a steady, sustainable rate.
        if (pace_us > 0) usleep(pace_us);

        uint64_t now = now_ms();
        if (now - last_report > 1000) {
            double sec = (now - start) / 1000.0;
            double mbps = (sent_bytes * 8.0 / 1e6) / sec;
            uint64_t nacks = wifi_get_nack_count() - nacks_at_start;

            // Sample RSSI. Note this issues a status query which costs a
            // UART round-trip; doing it only once per second keeps the
            // overhead negligible relative to send throughput.
            wifi_link_status_t st;
            int rssi_now = 0;
            bool rssi_ok = false;
            if (wifi_query_status(&st) == STATUS_SUCCESS) {
                rssi_now = st.rssi_dbm;
                rssi_sum += rssi_now;
                rssi_samples++;
                if (rssi_first) {
                    rssi_min = rssi_now;
                    rssi_max = rssi_now;
                    rssi_first = false;
                } else {
                    if (rssi_now < rssi_min) rssi_min = rssi_now;
                    if (rssi_now > rssi_max) rssi_max = rssi_now;
                }
                rssi_ok = true;
            }

            if (rssi_ok) {
                fprintf(stderr, "[test] %.2f MB sent, %.2f Mbps, %llu OK, %llu UART-fail, %llu nack, rssi=%d\n",
                        sent_bytes / 1048576.0, mbps,
                        (unsigned long long)sent_packets,
                        (unsigned long long)failed_packets,
                        (unsigned long long)nacks, rssi_now);
            } else {
                fprintf(stderr, "[test] %.2f MB sent, %.2f Mbps, %llu OK, %llu UART-fail, %llu nack\n",
                        sent_bytes / 1048576.0, mbps,
                        (unsigned long long)sent_packets,
                        (unsigned long long)failed_packets,
                        (unsigned long long)nacks);
            }
            last_report = now;

            // Abort early if NACKs are climbing fast. A high NACK rate
            // means the buoy is rejecting frames (TCP backpressure, or
            // worse, a desynced stream). Better to stop and diagnose
            // than to run for minutes producing junk numbers.
            // Threshold: >25% NACK rate sustained over at least 500 packets.
            if (sent_packets > 500 && nacks * 4 > sent_packets) {
                fprintf(stderr, "[test] NACK rate too high (%llu nack / %llu sent), aborting\n",
                        (unsigned long long)nacks, (unsigned long long)sent_packets);
                break;
            }
        }
    }

    // Drain the pipeline before stopping the clock. Without this, the
    // last few seconds of bytes are still queued in the UART/ESP32/TCP
    // pipeline when we compute throughput, inflating the Mbps number.
    // 5s is generous; TCP itself will time out long before that.
    fprintf(stderr, "[test] draining ESP32 pipeline...\n");
    Status drain_st = wifi_drain_pending(5000);
    if (drain_st != STATUS_SUCCESS) {
        fprintf(stderr, "[test] warning: drain timeout (bytes may still be in flight)\n");
    }

    uint64_t end = now_ms();
    double sec = (end - start) / 1000.0;
    double mbps = sec > 0 ? (sent_bytes * 8.0 / 1e6) / sec : 0;
    uint64_t nacks = wifi_get_nack_count() - nacks_at_start;
    double rssi_mean = (rssi_samples > 0) ? ((double)rssi_sum / rssi_samples) : 0.0;
    printf("\n=== SEND RESULTS ===\n");
    printf("Duration:        %.2f s (incl. pipeline drain)\n", sec);
    printf("Bytes sent:      %llu (%.2f MB)\n", (unsigned long long)sent_bytes, sent_bytes / 1048576.0);
    printf("Packets sent:    %llu\n", (unsigned long long)sent_packets);
    printf("UART failures:   %llu\n", (unsigned long long)failed_packets);
    printf("ESP32 NACKs:     %llu (asynchronous; not counted as failures)\n",
           (unsigned long long)nacks);
    printf("Throughput:      %.3f Mbps\n", mbps);
    if (rssi_samples > 0) {
        printf("RSSI (dBm):      mean=%.1f  min=%d  max=%d  (n=%d samples)\n",
               rssi_mean, rssi_min, rssi_max, rssi_samples);
    } else {
        printf("RSSI (dBm):      no samples collected\n");
    }

    if (output_path && output_path[0]) {
        FILE *fp = fopen(output_path, "w");
        if (!fp) {
            fprintf(stderr, "[test] WARNING: could not write to %s\n", output_path);
        } else {
            char ts[64];
            format_timestamp(ts, sizeof(ts));
            fprintf(fp, "# WiFi ATP test results (send side)\n");
            fprintf(fp, "# Generated %s\n", ts);
            fprintf(fp, "\n[results]\n");
            fprintf(fp, "mode = send\n");
            fprintf(fp, "timestamp = %s\n", ts);
            fprintf(fp, "duration_sec = %.3f\n", sec);
            fprintf(fp, "bytes_sent = %llu\n", (unsigned long long)sent_bytes);
            fprintf(fp, "bytes_sent_mb = %.3f\n", sent_bytes / 1048576.0);
            fprintf(fp, "packets_sent = %llu\n", (unsigned long long)sent_packets);
            fprintf(fp, "packet_size_bytes = %u\n", pkt_size);
            fprintf(fp, "uart_failures = %llu\n", (unsigned long long)failed_packets);
            fprintf(fp, "esp32_nacks = %llu\n", (unsigned long long)nacks);
            fprintf(fp, "pace_us = %u\n", pace_us);
            fprintf(fp, "throughput_mbps = %.4f\n", mbps);
            fprintf(fp, "rssi_samples = %d\n", rssi_samples);
            if (rssi_samples > 0) {
                fprintf(fp, "rssi_mean_dbm = %.2f\n", rssi_mean);
                fprintf(fp, "rssi_min_dbm = %d\n", rssi_min);
                fprintf(fp, "rssi_max_dbm = %d\n", rssi_max);
            }
            fprintf(fp, "\n[plaintext]\n");
            fprintf(fp, "=== SEND RESULTS ===\n");
            fprintf(fp, "Duration:        %.2f s (incl. pipeline drain)\n", sec);
            fprintf(fp, "Bytes sent:      %llu (%.2f MB)\n",
                    (unsigned long long)sent_bytes, sent_bytes / 1048576.0);
            fprintf(fp, "Packets sent:    %llu\n", (unsigned long long)sent_packets);
            fprintf(fp, "UART failures:   %llu\n", (unsigned long long)failed_packets);
            fprintf(fp, "ESP32 NACKs:     %llu\n", (unsigned long long)nacks);
            fprintf(fp, "Throughput:      %.3f Mbps\n", mbps);
            if (rssi_samples > 0) {
                fprintf(fp, "RSSI:            mean=%.1f dBm, min=%d dBm, max=%d dBm (n=%d)\n",
                        rssi_mean, rssi_min, rssi_max, rssi_samples);
            }
            fclose(fp);
            fprintf(stderr, "[test] results written to %s\n", output_path);
        }
    }

    free(buf);
    return 0;
}

static int run_recv(double max_duration_sec, uint64_t expected_bytes,
                    const char *output_path) {
    // Idle timeout: if no packet arrives within this window AFTER the
    // first packet has been received, declare the sender done and stop
    // the test. Chosen to be longer than any plausible Wi-Fi transient
    // (retries, brief AP scans) but short enough that the test exits
    // promptly when the buoy genuinely stops.
    const uint32_t IDLE_TIMEOUT_MS = 3000;

    fprintf(stderr, "[test] receiving (max duration %.1fs, expected_bytes=%llu, idle timeout %u ms)\n",
            max_duration_sec, (unsigned long long)expected_bytes, IDLE_TIMEOUT_MS);

    uint8_t buf[MAX_WIFI_RECV_PACKET_LEN];
    uint64_t srcmac = 0;
    BufLen len = 0;

    uint64_t total_bytes = 0;
    uint64_t total_pkts = 0;
    uint64_t first_pkt_time = 0;
    uint64_t start_wallclock = now_ms();
    uint64_t last_report = start_wallclock;

    // Loop until any of:
    //   - expected_bytes received
    //   - max_duration_sec exceeded
    //   - IDLE_TIMEOUT_MS of silence after the first packet (buoy stopped)
    while (1) {
        // Before each iteration, decide how long to wait for the next
        // packet. If we have already received at least one packet, use
        // the idle timeout. If we haven't, wait up to the remaining
        // max_duration so the test doesn't exit before the buoy starts
        // sending. The reader thread does the actual UART read; this
        // call just blocks on a condvar with a deadline.
        uint64_t now = now_ms();
        uint32_t wait_ms;
        if (total_pkts == 0) {
            double elapsed_sec = (now - start_wallclock) / 1000.0;
            double remaining_sec = max_duration_sec - elapsed_sec;
            if (remaining_sec <= 0) {
                fprintf(stderr, "[test] hit max duration before any packets arrived\n");
                break;
            }
            // Cap the per-call wait so we periodically check for max
            // duration even if no packets arrive at all.
            wait_ms = (remaining_sec * 1000.0 > 1000.0) ? 1000
                                                         : (uint32_t)(remaining_sec * 1000.0);
        } else {
            wait_ms = IDLE_TIMEOUT_MS;
        }

        Status s = recv_wifi_packet_timeout(&srcmac, buf, &len, wait_ms);
        if (s == STATUS_SUCCESS) {
            if (total_pkts == 0) first_pkt_time = now_ms();
            total_bytes += len;
            total_pkts++;
        } else if (s == STATUS_MODULE_DETACHED) {
            fprintf(stderr, "[test] recv error: module detached\n");
            break;
        } else {
            // Timeout. If we've received at least one packet, this means
            // the buoy stopped sending — clean exit.
            if (total_pkts > 0) {
                fprintf(stderr, "[test] idle %u ms with no new packets; declaring sender done\n",
                        IDLE_TIMEOUT_MS);
                break;
            }
            // No packets yet — fall through to check max_duration on next loop.
        }

        // Refresh `now` for the periodic-report and end-of-loop checks
        // (it was set near the top of the iteration before we waited).
        now = now_ms();
        if (total_pkts > 0 && now - last_report > 1000) {
            double sec = (now - first_pkt_time) / 1000.0;
            double mbps = sec > 0 ? (total_bytes * 8.0 / 1e6) / sec : 0;
            // Reader-thread side: bytes the laptop's reader thread has
            // pulled out of the UART (MSG_RECV frames). If this number
            // is climbing but total_bytes (this loop) is not, the test
            // code is the bottleneck. If reader-thread is also flat,
            // the ESP32 isn't forwarding.
            uint64_t rdr_frames = wifi_get_recv_frame_count();
            uint64_t rdr_bytes  = wifi_get_recv_bytes_count();
            fprintf(stderr,
                    "[test] %.2f MB recv, %.2f Mbps "
                    "(reader-thread saw %llu frames / %.2f MB)",
                    total_bytes / 1048576.0, mbps,
                    (unsigned long long)rdr_frames, rdr_bytes / 1048576.0);
            // Only print source MAC if it's actually populated. UDP mode
            // leaves it zero because the firmware doesn't expose the
            // peer MAC over the UDP API; printing all zeros is noise.
            if (srcmac != 0) {
                uint8_t m[6];
                uint64_t tmp = srcmac;
                for (int i = 5; i >= 0; i--) { m[i] = (uint8_t)(tmp & 0xFF); tmp >>= 8; }
                fputs(" (last src ", stderr);
                print_mac(m);
                fputc(')', stderr);
            }
            fputc('\n', stderr);
            last_report = now;
        }
        if (expected_bytes > 0 && total_bytes >= expected_bytes) break;
        if ((now - start_wallclock) / 1000.0 > max_duration_sec) {
            fprintf(stderr, "[test] hit max duration\n");
            break;
        }
    }

    uint64_t end = now_ms();
    double sec = first_pkt_time > 0 ? (end - first_pkt_time) / 1000.0 : 0;
    double mbps = sec > 0 ? (total_bytes * 8.0 / 1e6) / sec : 0;
    uint64_t rdr_frames = wifi_get_recv_frame_count();
    uint64_t rdr_bytes  = wifi_get_recv_bytes_count();
    printf("\n=== RECV RESULTS ===\n");
    printf("Duration:        %.2f s\n", sec);
    printf("Bytes received:  %llu (%.2f MB)\n", (unsigned long long)total_bytes, total_bytes / 1048576.0);
    printf("Packets:         %llu\n", (unsigned long long)total_pkts);
    printf("Reader-thread:   %llu frames, %.2f MB (compare with above)\n",
           (unsigned long long)rdr_frames, rdr_bytes / 1048576.0);
    printf("Throughput:      %.3f Mbps\n", mbps);

    if (output_path && output_path[0]) {
        FILE *fp = fopen(output_path, "w");
        if (!fp) {
            fprintf(stderr, "[test] WARNING: could not write to %s\n", output_path);
        } else {
            char ts[64];
            format_timestamp(ts, sizeof(ts));
            // Compute delivery rate vs expected (if expected_bytes was set,
            // i.e. this is matching a known send target).
            double delivery_pct = 0.0;
            if (expected_bytes > 0) {
                delivery_pct = 100.0 * (double)total_bytes / (double)expected_bytes;
            }
            fprintf(fp, "# WiFi ATP test results (recv side)\n");
            fprintf(fp, "# Generated %s\n", ts);
            fprintf(fp, "\n[results]\n");
            fprintf(fp, "mode = recv\n");
            fprintf(fp, "timestamp = %s\n", ts);
            fprintf(fp, "duration_sec = %.3f\n", sec);
            fprintf(fp, "bytes_received = %llu\n", (unsigned long long)total_bytes);
            fprintf(fp, "bytes_received_mb = %.3f\n", total_bytes / 1048576.0);
            fprintf(fp, "expected_bytes = %llu\n", (unsigned long long)expected_bytes);
            if (expected_bytes > 0) {
                fprintf(fp, "delivery_pct = %.2f\n", delivery_pct);
            }
            fprintf(fp, "packets = %llu\n", (unsigned long long)total_pkts);
            fprintf(fp, "reader_thread_frames = %llu\n", (unsigned long long)rdr_frames);
            fprintf(fp, "reader_thread_bytes = %llu\n", (unsigned long long)rdr_bytes);
            fprintf(fp, "throughput_mbps = %.4f\n", mbps);
            fprintf(fp, "\n[plaintext]\n");
            fprintf(fp, "=== RECV RESULTS ===\n");
            fprintf(fp, "Duration:        %.2f s\n", sec);
            fprintf(fp, "Bytes received:  %llu (%.2f MB)\n",
                    (unsigned long long)total_bytes, total_bytes / 1048576.0);
            fprintf(fp, "Packets:         %llu\n", (unsigned long long)total_pkts);
            fprintf(fp, "Reader-thread:   %llu frames, %.2f MB\n",
                    (unsigned long long)rdr_frames, rdr_bytes / 1048576.0);
            fprintf(fp, "Throughput:      %.3f Mbps\n", mbps);
            if (expected_bytes > 0) {
                fprintf(fp, "Delivery:        %.2f%% of expected %.2f MB\n",
                        delivery_pct, expected_bytes / 1048576.0);
            }
            fclose(fp);
            fprintf(stderr, "[test] results written to %s\n", output_path);
        }
    }

    return 0;
}

static int run_echo(void) {
    fprintf(stderr, "[test] echo mode — receive packets, echo back to sender\n");
    uint8_t buf[MAX_WIFI_RECV_PACKET_LEN];
    uint64_t srcmac;
    BufLen len;
    uint64_t n = 0;
    while (1) {
        Status s = recv_wifi_packet(&srcmac, buf, &len);
        if (s != STATUS_SUCCESS) { fprintf(stderr, "[test] recv error\n"); return -1; }
        n++;
        send_wifi_packet(srcmac, buf, len);
        if (n % 100 == 0) fprintf(stderr, "[test] echoed %llu packets\n", (unsigned long long)n);
    }
}

static int run_atp02(const char *side, uint64_t override_bytes, uint32_t pace_us,
                     const char *output_path) {
    // ATP02: throughput test. Default is 1 MB for quick iteration; use
    // --bytes to override (e.g. --bytes 10485760 for 10 MB).
    uint64_t target = (override_bytes > 0) ? override_bytes : 1ULL * 1024 * 1024;
    if (strcmp(side, "ship") == 0) {
        // Receive `target` bytes with a generous duration cap.
        // The recv side stops when target bytes have been counted.
        return run_recv(300.0, target, output_path);
    } else {
        // Send `target` bytes. Destination is broadcast — buoy connects
        // to one peer (the ship) so the broadcast resolves to that peer.
        return run_send(target, MAX_WIFI_SEND_PACKET_LEN, 0xFFFFFFFFFFFFULL,
                        pace_us, output_path);
    }
}

static int run_atp05(int cycles, uint32_t echo_timeout_ms) {
    // ATP05: sleep -> power cycle -> wake -> send test message -> verify echo.
    //
    // Run this on the buoy side. The ship must be running:
    //   ./test_wifi --mode echo
    // so it bounces our messages back over the Wi-Fi link.
    //
    // A cycle counts as successful only if all of the following hold:
    //   (a) shutdown_wifi_module() returns
    //   (b) initialize_wifi_module() returns STATUS_SUCCESS
    //   (c) send_wifi_packet() returns STATUS_SUCCESS for the test message
    //   (d) recv_wifi_packet_timeout() returns STATUS_SUCCESS within
    //       echo_timeout_ms, AND the echoed payload matches what we sent
    //
    // (d) is the key check: it proves the radio actually came back online
    // and a TCP-level round trip works, not merely that the local UART
    // accepted a frame.
    fprintf(stderr, "[test] ATP05: %d sleep/wake cycles, echo timeout %u ms\n",
            cycles, echo_timeout_ms);

    int successes = 0;
    int hard_failures = 0;  // distinct from cycles that completed but didn't verify

    for (int i = 0; i < cycles; i++) {
        fprintf(stderr, "\n--- Cycle %d/%d ---\n", i + 1, cycles);

        // (a) Shut down
        fprintf(stderr, "[test] shutdown_wifi_module()\n");
        shutdown_wifi_module();
        sleep(2);

        fprintf(stderr, "[test] >>> POWER CYCLE THE BUOY NOW (5 second window) <<<\n");
        sleep(5);

        // (b) Re-initialize
        fprintf(stderr, "[test] initialize_wifi_module()\n");
        Status s = initialize_wifi_module();
        if (s != STATUS_SUCCESS) {
            fprintf(stderr, "[test] cycle %d FAIL: init returned %d\n", i + 1, s);
            hard_failures++;
            continue;
        }

        // Build a per-cycle test payload. Including the cycle index and a
        // monotonic timestamp as a nonce means a stale echo from a previous
        // cycle cannot satisfy the verification below even if drain misses it.
        uint8_t msg[64];
        int msg_len = snprintf((char*)msg, sizeof(msg),
                               "ATP05 cycle=%d nonce=%llu",
                               i + 1, (unsigned long long)now_ms());
        if (msg_len < 0 || msg_len >= (int)sizeof(msg)) {
            fprintf(stderr, "[test] cycle %d FAIL: snprintf error\n", i + 1);
            hard_failures++;
            continue;
        }

        // Drain any old packets buffered while the link was down.
        wifi_drain_recv_queue();

        // (c) Send test message to the ship (broadcast MAC — ship has
        // exactly one TCP peer connected at a time).
        s = send_wifi_packet(0xFFFFFFFFFFFFULL, msg, (BufLen)msg_len);
        if (s != STATUS_SUCCESS) {
            fprintf(stderr, "[test] cycle %d FAIL: send returned %d\n", i + 1, s);
            continue;
        }
        fprintf(stderr, "[test] cycle %d: sent %d bytes, waiting for echo...\n",
                i + 1, msg_len);

        // (d) Wait for echo and verify.
        // The ship's --mode echo bounces our payload back unchanged.
        // We may receive more than one packet here if some unrelated traffic
        // arrives (shouldn't, but be defensive) — loop until we get one that
        // matches our nonce, or the cumulative timeout expires.
        uint64_t deadline = now_ms() + echo_timeout_ms;
        bool verified = false;

        while (!verified) {
            uint64_t now = now_ms();
            if (now >= deadline) {
                fprintf(stderr, "[test] cycle %d FAIL: no matching echo within %u ms\n",
                        i + 1, echo_timeout_ms);
                break;
            }
            uint64_t remain_ms = deadline - now;

            uint64_t srcmac = 0;
            BufLen rlen = 0;
            uint8_t recvbuf[MAX_WIFI_RECV_PACKET_LEN];

            Status rs = recv_wifi_packet_timeout(&srcmac, recvbuf, &rlen,
                                                 (uint32_t)remain_ms);
            if (rs != STATUS_SUCCESS) {
                // Timeout (or driver error). Either way, this cycle fails.
                fprintf(stderr, "[test] cycle %d FAIL: no echo within %u ms\n",
                        i + 1, echo_timeout_ms);
                break;
            }

            if (rlen == (BufLen)msg_len && memcmp(recvbuf, msg, msg_len) == 0) {
                verified = true;
                fprintf(stderr, "[test] cycle %d OK: echo matched (%d bytes, src=",
                        i + 1, msg_len);
                uint8_t mb[6];
                for (int k = 5; k >= 0; k--) { mb[k] = (uint8_t)(srcmac & 0xFF); srcmac >>= 8; }
                print_mac(mb);
                fputs(")\n", stderr);
            } else {
                // Wrong packet — log and keep waiting until deadline.
                fprintf(stderr, "[test] cycle %d: discarded non-matching packet "
                                "(len=%u expected=%d), still waiting...\n",
                        i + 1, (unsigned)rlen, msg_len);
            }
        }

        if (verified) {
            successes++;
        }
    }

    printf("\n=== ATP05 RESULTS ===\n");
    printf("Cycles requested:     %d\n", cycles);
    printf("Cycles verified:      %d\n", successes);
    printf("Init/setup failures:  %d\n", hard_failures);
    printf("Send/echo failures:   %d\n", cycles - successes - hard_failures);
    printf("Pass criterion:       all %d cycles must verify\n", cycles);
    printf("Result:               %s\n", (successes == cycles) ? "PASS" : "FAIL");
    return (successes == cycles) ? 0 : 1;
}

static int run_status(void) {
    wifi_link_status_t st;
    Status s = wifi_query_status(&st);
    if (s != STATUS_SUCCESS) { fprintf(stderr, "[test] status query failed\n"); return -1; }
    printf("Connected: %s\n", st.connected ? "yes" : "no");
    printf("RSSI:      %d dBm\n", st.rssi_dbm);
    printf("Self MAC:  ");
    print_mac(st.self_mac);
    printf("\n");
    return 0;
}

// ---------------- main ---------------- //

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s --atp 02 --side ship|buoy [--bytes N] [--pace-us US] [--output FILE]\n"
        "  %s --atp 05 --side buoy --cycles N [--timeout MS]\n"
        "  %s --mode send  [--bytes N] [--pkt-size N] [--destmac HEX] [--pace-us US] [--output FILE]\n"
        "  %s --mode recv  [--bytes N] [--duration SEC] [--output FILE]\n"
        "  %s --mode echo\n"
        "  %s --mode status\n"
        "\n"
        "Options:\n"
        "  --pace-us US    inter-frame delay on send side (default 1500; use\n"
        "                  10000 for reliable transfer with current firmware)\n"
        "  --output FILE   write structured results to FILE on exit\n"
        "\n"
        "Environment variables:\n"
        "  WIFI_SERIAL_DEVICE   path to the ESP32's serial device\n"
        "  WIFI_ROLE            ship|buoy (must match ESP32's firmware)\n",
        prog, prog, prog, prog, prog, prog);
}

int main(int argc, char **argv) {
    const char *atp = NULL;
    const char *mode = NULL;
    const char *side = NULL;
    uint64_t bytes = 1ULL * 1024 * 1024;  // default 1 MB
    bool bytes_set = false;                 // distinguishes default vs explicit
    uint16_t pkt_size = MAX_WIFI_SEND_PACKET_LEN;
    uint64_t destmac = 0xFFFFFFFFFFFFULL;
    int cycles = 3;
    double duration = 300.0;
    uint32_t atp05_echo_timeout_ms = 10000;
    // Default pace: 1500us per frame ~= 666 fps. At 506 B/frame that's
    // ~2.7 Mbps "ideal" — well above UART line rate. The kernel/UART
    // will queue the excess and effective rate becomes UART-bound. The
    // small sleep prevents the laptop from getting *very* far ahead of
    // the wire (which we saw cause ESP32 UART RX overflow).
    uint32_t pace_us = 1500;
    const char *output_path = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--atp") && i+1 < argc) atp = argv[++i];
        else if (!strcmp(argv[i], "--mode") && i+1 < argc) mode = argv[++i];
        else if (!strcmp(argv[i], "--side") && i+1 < argc) side = argv[++i];
        else if (!strcmp(argv[i], "--bytes") && i+1 < argc) {
            bytes = strtoull(argv[++i], NULL, 10);
            bytes_set = true;
        }
        else if (!strcmp(argv[i], "--pkt-size") && i+1 < argc) pkt_size = (uint16_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--destmac") && i+1 < argc) destmac = strtoull(argv[++i], NULL, 16);
        else if (!strcmp(argv[i], "--cycles") && i+1 < argc) cycles = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--duration") && i+1 < argc) duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--timeout") && i+1 < argc)
            atp05_echo_timeout_ms = (uint32_t)strtoul(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "--pace-us") && i+1 < argc)
            pace_us = (uint32_t)strtoul(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "--output") && i+1 < argc)
            output_path = argv[++i];
        else { usage(argv[0]); return 1; }
    }

    if (!atp && !mode) { usage(argv[0]); return 1; }

    if (!is_wifi_module_attached()) {
        fprintf(stderr, "WiFi module not detected on serial port. Set WIFI_SERIAL_DEVICE.\n");
        return 1;
    }

    Status s = initialize_wifi_module();
    if (s != STATUS_SUCCESS) {
        fprintf(stderr, "initialize_wifi_module failed (status=%d)\n", s);
        return 1;
    }
    fprintf(stderr, "[test] initialized\n");

    int rc = 0;
    if (atp) {
        if (!side) { fprintf(stderr, "--atp requires --side ship|buoy\n"); rc = 1; }
        else if (!strcmp(atp, "02")) rc = run_atp02(side, bytes_set ? bytes : 0, pace_us, output_path);
        else if (!strcmp(atp, "05")) rc = run_atp05(cycles, atp05_echo_timeout_ms);
        else { fprintf(stderr, "unknown ATP %s\n", atp); rc = 1; }
    } else {
        if (!strcmp(mode, "send"))   rc = run_send(bytes, pkt_size, destmac, pace_us, output_path);
        else if (!strcmp(mode, "recv"))  rc = run_recv(duration, bytes, output_path);
        else if (!strcmp(mode, "echo"))  rc = run_echo();
        else if (!strcmp(mode, "status")) rc = run_status();
        else { fprintf(stderr, "unknown mode %s\n", mode); rc = 1; }
    }

    shutdown_wifi_module();
    return rc;
}