// test_lora.c
//
// Standalone test program for the LoRa driver.
//
// Usage:
//   test_lora send <device>     - configure + transmit a test packet every 2 sec
//   test_lora recv <device>     - configure + listen, print received packets
//
// Example (two CP2102s on same Mac):
//   Terminal A:  ./test_lora send /dev/cu.usbserial-0001
//   Terminal B:  ./test_lora recv /dev/cu.usbserial-0002
//
// Build: make

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
#include <unistd.h>
#include "../include/drivers.h"

static void print_bytes(const uint8_t *data, BufLen len) {
    for (BufLen i = 0; i < len; i++) {
        printf("%02X", data[i]);
    }
}

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
        // Embed a counter so the receiver can tell packets apart.
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
        Status s = recv_lora_packet(data, &len, 30000);  // 30s timeout
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
        printf("  (\"%.*s\")\n", (int)len, (char *)data);
        fflush(stdout);
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s {send|recv} [device]\n", argv[0]);
        fprintf(stderr, "  device defaults to LORA_SERIAL_DEVICE env or /dev/cu.usbserial-0001\n");
        return 1;
    }

    // Allow device override via positional arg.
    if (argc >= 3) {
        setenv("LORA_SERIAL_DEVICE", argv[2], 1);
    }

    if (strcmp(argv[1], "send") == 0) return do_send();
    if (strcmp(argv[1], "recv") == 0) return do_recv();

    fprintf(stderr, "unknown mode: %s\n", argv[1]);
    return 1;
}
