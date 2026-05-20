#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20

#define BAUD_RATE    115200
#define MAX_PAYLOAD  250
#define ESPNOW_CHAN  1

// SLIP special bytes (RFC 1055)
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC   // sent after ESC to represent 0xC0
#define SLIP_ESC_ESC 0xDD   // sent after ESC to represent 0xDB

// Laptop <-> serial(SLIP) <-> ESP32 <-> ESP-NOW <-> ESP32 <-> serial(SLIP) <-> embedded sys
//
// Laptop side  : USB-C (Serial)
// Embedded side: UART1 on pins TX=21, RX=20
//
// Serial framing : SLIP (RFC 1055).
//                  Each packet is delimited by 0xC0 (END) bytes.
//                  0xC0 in payload → 0xDB 0xDC
//                  0xDB in payload → 0xDB 0xDD
//                  A leading END before each packet flushes any line garbage.
//
// ESP-NOW        : one esp_now_send() per decoded packet; boundaries preserved natively.
//                  Received ESP-NOW datagrams are forwarded to serial with SLIP framing.

// Comment out for embedded-side build
#define LAPTOP_SIDE

// #define EMBEDDED_SIDE_DEBUG

uint8_t MAC_A[] = {0x08, 0x92, 0x72, 0x85, 0x8B, 0xF0};
uint8_t MAC_B[] = {0x08, 0x92, 0x72, 0x85, 0x0B, 0xB8};

#ifdef LAPTOP_SIDE
  #define CorrectSerial Serial
  #define SETUP_SERIAL (Serial.begin(BAUD_RATE))
  #define PEER_MAC MAC_B
#else
  #ifdef EMBEDDED_SIDE_DEBUG
    #define CorrectSerial Serial
    #define SETUP_SERIAL (Serial.begin(BAUD_RATE))
  #else
    #define CorrectSerial Serial1
    #define SETUP_SERIAL (Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN))
  #endif
  #define PEER_MAC MAC_A
#endif

// ── SLIP TX ───────────────────────────────────────────────────────────────────

// Write one SLIP-framed packet to serial.
// Leading END flushes any partial garbage the receiver may have accumulated.
void slip_send(const uint8_t *data, size_t len) {
    uint8_t b;

    b = SLIP_END; CorrectSerial.write(&b, 1);          // leading END (line flush)

    for (size_t i = 0; i < len; i++) {
        if (data[i] == SLIP_END) {
            b = SLIP_ESC; CorrectSerial.write(&b, 1);
            b = SLIP_ESC_END; CorrectSerial.write(&b, 1);
        } else if (data[i] == SLIP_ESC) {
            b = SLIP_ESC; CorrectSerial.write(&b, 1);
            b = SLIP_ESC_ESC; CorrectSerial.write(&b, 1);
        } else {
            CorrectSerial.write(&data[i], 1);
        }
    }

    b = SLIP_END; CorrectSerial.write(&b, 1);          // trailing END
}
