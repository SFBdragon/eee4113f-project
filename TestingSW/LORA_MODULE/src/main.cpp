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


#include <WiFi.h>
#include <esp_wifi.h>

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

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
