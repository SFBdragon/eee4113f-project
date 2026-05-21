#include <Arduino.h>
#include <WiFi.h>
#include <stdint.h>
#include <string.h>
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


uint8_t PEER_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#define LED_PIN 8
#define LED_ON  0
#define LED_OFF 1

// ── Callbacks ─────────────────────────────────────────────────────────────────

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    // See original note: avoid writing to CorrectSerial here.
}

uint32_t on_receive_flag = 0;

// Received a packet over ESP-NOW → SLIP-frame it and write to local serial
void onRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len <= 0 || len > MAX_PAYLOAD) return;
    Serial.write(data, len);
    on_receive_flag = 1;
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(BAUD_RATE);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        while (true) delay(200);
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, PEER_MAC, 6);
    peer.channel = ESPNOW_CHAN;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        while (true) delay(200);
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_ON);
}

void loop() {
    if (on_receive_flag) {
        on_receive_flag = 0;

        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, LED_OFF);
            delay(100);
            digitalWrite(LED_PIN, LED_ON);
            delay(100);
        }
    }

    uint8_t b;
    if (Serial.available()) {
        b = (uint8_t)Serial.read();
    } else {
        return;
    }

    esp_now_send(PEER_MAC, &b, 1);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LED_OFF);
        delay(100);
        digitalWrite(LED_PIN, LED_ON);
        delay(100);
    }
}

