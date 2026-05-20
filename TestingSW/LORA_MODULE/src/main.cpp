<<<<<<< HEAD
=======
// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_now.h>

<<<<<<< HEAD
#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20
#define ONBOARD_LED   8

#define BAUD_RATE         115200
#define SEND_INTERVAL_MS  1000

#define NUM_LETTERS  4   // <-- CHANGE THIS: how many different letters to cycle (max 26)
#define REPEAT_COUNT 4   // <-- CHANGE THIS: how many times each letter repeats per packet

// ─── RX ───────────────────────────────────────────────────────────────────────

#define RX_BUF_SIZE 128
uint8_t  rxBuf[RX_BUF_SIZE];
uint8_t  rxLen        = 0;
uint32_t lastByteTime = 0;
#define  PACKET_TIMEOUT_MS 30

void printRxBuffer() {
  if (rxLen == 0) return;
  Serial.print("<<< RX (");
  Serial.print(rxLen);
  Serial.print(" bytes): ");
  for (uint8_t i = 0; i < rxLen; i++) {
    Serial.print((char)rxBuf[i]);
  }
  Serial.println();
  rxLen = 0;
}

// ─── SETUP / LOOP ─────────────────────────────────────────────────────────────

uint8_t  letterIndex  = 0;
uint32_t lastSendTime = 0;

void setup() {
  pinMode(ONBOARD_LED, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
  Serial.println("=== Ready ===");
=======
// // ─── CONFIGURATION ───────────────────────────────────────────────────────────
// #define BAUD_RATE 115200
// #define SERIAL_TX_PIN 21
// #define SERIAL_RX_PIN 20

// // Put the MAC address of the OTHER ESP32 here
// uint8_t REMOTE_MAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; 

// // Tuning parameters for serial packing
// #define MAX_ESP_NOW_PAYLOAD 250   // ESP-NOW hard hardware limit is 250 bytes
// #define SERIAL_TIMEOUT_MS 4       // Wait up to 4ms for a gap in serial streaming

// uint8_t serialBuffer[MAX_ESP_NOW_PAYLOAD];
// uint8_t bufferIdx = 0;
// uint32_t lastByteTime = 0;

// // ─── ESP-NOW CALLBACKS ───────────────────────────────────────────────────────

// // Triggered automatically when a wireless packet arrives
// void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
//   // If you want to strictly filter by sender MAC, you can inspect recvInfo->src_addr here
  
//   // Directly stream wireless payload out of the physical serial port
//   Serial1.write(incomingData, len);
// }

// // Optional: Status callback to verify wireless delivery success
// void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   // status == ESP_NOW_SEND_SUCCESS means the remote ESP32 acknowledged the frame
// }

// // ─── SETUP ────────────────────────────────────────────────────────────────────

// void setup() {
//   // Initialize Serial1 for your device communications
//   Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

//   // Initialize Wi-Fi in Station Mode (required for ESP-NOW)
//   WiFi.mode(WIFI_STA);
//   WiFi.disconnect(); // Do not connect to a standard home router

//   // Initialize ESP-NOW Protocol
//   if (esp_now_init() != ESP_OK) {
//     Serial1.println("Error initializing ESP-NOW");
//     return;
//   }

//   // Register the structural callbacks
//   esp_now_register_recv_cb(onDataRecv);
//   esp_now_register_send_cb(onDataSent);

//   // Register the peer (the twin ESP32)
//   esp_now_peer_info_t peerInfo = {};
//   memcpy(peerInfo.peer_addr, REMOTE_MAC, 6);
//   peerInfo.channel = 1;     // Must match on both devices
//   peerInfo.encrypt = false; // Set true if you want hardware-level encryption

//   if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//     Serial1.println("Failed to add peer");
//     return;
//   }
// }

// // ─── LOOP ─────────────────────────────────────────────────────────────────────

// void loop() {
//   // Read physical serial data into our wireless buffer
//   while (Serial1.available() && bufferIdx < MAX_ESP_NOW_PAYLOAD) {
//     serialBuffer[bufferIdx++] = Serial1.read();
//     lastByteTime = millis();
//   }

//   // Determine if it's time to transmit the wireless frame
//   if (bufferIdx > 0) {
//     bool bufferFull = (bufferIdx >= MAX_ESP_NOW_PAYLOAD);
//     bool serialStreamGap = ((millis() - lastByteTime) > SERIAL_TIMEOUT_MS);

//     if (bufferFull || serialStreamGap) {
//       // Fire packet connectionless-ly to the pre-registered peer address
//       esp_err_t result = esp_now_send(REMOTE_MAC, serialBuffer, bufferIdx);
      
//       // Reset the buffer tracker immediately
//       bufferIdx = 0;
//     }
//   }
// }

>>>>>>> c7217bb77c9a6fc445300e92fc5fcb44f41263cc
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

<<<<<<< HEAD
<<<<<<< HEAD
// ── Callbacks ─────────────────────────────────────────────────────────────────

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    // See original note: avoid writing to CorrectSerial here.
}

// Received a packet over ESP-NOW → SLIP-frame it and write to local serial
void onRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len <= 0 || len > MAX_PAYLOAD) return;
    slip_send(data, (size_t)len);
}

// ── Setup ─────────────────────────────────────────────────────────────────────

#define LED_PIN 8
#define LED_ON  0
#define LED_OFF 1

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    SETUP_SERIAL;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        while (true) delay(1000);
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, PEER_MAC, 6);
    peer.channel = ESPNOW_CHAN;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        while (true) delay(1000);
    }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
//
// SLIP RX state machine:
//   IDLE     : between packets, waiting for END or first data byte
//   IN_PACKET: accumulating payload bytes
//   IN_ESC   : last byte was ESC, next byte is the escaped value

static enum { IDLE, IN_PACKET, IN_ESC } rx_state = IDLE;
static uint8_t rx_buf[MAX_PAYLOAD];
static size_t  rx_count = 0;

// Called when a complete SLIP packet has been received into rx_buf[0..len-1]
static void dispatch_packet(size_t len) {
    if (len == 0) return;   // zero-length packet: valid SLIP, nothing to send

    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LED_ON);
        delay(100);
        digitalWrite(LED_PIN, LED_OFF);
        delay(100);
    }

    esp_now_send(PEER_MAC, rx_buf, len);
}

void loop() {
    while (CorrectSerial.available()) {
        uint8_t b = (uint8_t)CorrectSerial.read();

        switch (rx_state) {

            case IDLE:
                if (b == SLIP_END) {
                    // Another END while idle: still idle (double-END gap packet)
                    break;
                }
                // First non-END byte starts a packet
                rx_count = 0;
                if (b == SLIP_ESC) {
                    rx_state = IN_ESC;
                } else {
                    if (rx_count < MAX_PAYLOAD) rx_buf[rx_count++] = b;
                    rx_state = IN_PACKET;
                }
                break;

            case IN_PACKET:
                if (b == SLIP_END) {
                    dispatch_packet(rx_count);
                    rx_count = 0;
                    rx_state = IDLE;
                } else if (b == SLIP_ESC) {
                    rx_state = IN_ESC;
                } else {
                    if (rx_count < MAX_PAYLOAD) rx_buf[rx_count++] = b;
                    // silently drop bytes beyond MAX_PAYLOAD; packet will be truncated
                }
                break;

            case IN_ESC:
                if (b == SLIP_ESC_END) {
                    if (rx_count < MAX_PAYLOAD) rx_buf[rx_count++] = SLIP_END;
                } else if (b == SLIP_ESC_END) {
                    if (rx_count < MAX_PAYLOAD) rx_buf[rx_count++] = SLIP_ESC;
                } else {
                    // Protocol violation: ESC followed by non-escape byte.
                    // RFC 1055 doesn't specify; discard the ESC, treat byte literally.
                    if (rx_count < MAX_PAYLOAD) rx_buf[rx_count++] = b;
                }
                rx_state = IN_PACKET;
                break;
        }
    }
}
=======
<<<<<<< HEAD
=======
>>>>>>> ea0e02733f0fd5abee21d5058c2a828e14036903
void setup(){
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
>>>>>>> e31b850ded5af5febd80a7a5aa5aa42e168bf03f
}
 
void loop(){

<<<<<<< HEAD
void loop() {

  // ── RX ─────────────────────────────────────────────────────────────────────
  while (Serial1.available()) {
    if (rxLen < RX_BUF_SIZE) rxBuf[rxLen++] = Serial1.read();
    else Serial1.read();
    lastByteTime = millis();
  }
  if (rxLen > 0 && (millis() - lastByteTime) > PACKET_TIMEOUT_MS) {
    printRxBuffer();
  }

  // ── TX ─────────────────────────────────────────────────────────────────────
  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    digitalWrite(ONBOARD_LED, HIGH);

    char letter = 'A' + letterIndex;

    // Build and send packet
    for (uint8_t i = 0; i < REPEAT_COUNT; i++) Serial1.write(letter);
    Serial1.write('\n');
    Serial1.flush();

    // Log to USB serial
    Serial.print(">>> TX: ");
    for (uint8_t i = 0; i < REPEAT_COUNT; i++) Serial.print(letter);
    Serial.println();

    letterIndex = (letterIndex + 1) % NUM_LETTERS;
    lastSendTime = millis();
    digitalWrite(ONBOARD_LED, LOW);
  }
}
=======
}
>>>>>>> e31b850ded5af5febd80a7a5aa5aa42e168bf03f
=======
void loop() {}
>>>>>>> 388195328feb5b86c91b5f26495e8369952d1c79
<<<<<<< HEAD
>>>>>>> c7217bb77c9a6fc445300e92fc5fcb44f41263cc
=======
>>>>>>> ea0e02733f0fd5abee21d5058c2a828e14036903
