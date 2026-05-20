// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_now.h>

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

#include <Arduino.h>
#include "esp_wifi.h"

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Initialize Wi-Fi in station mode (required to read MAC)
    esp_wifi_init(nullptr);  // uses default config
    esp_wifi_set_mode(WIFI_MODE_STA);

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    Serial.printf("STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Base MAC (factory-burned into eFuse; STA derives from this)
    uint8_t base_mac[6];
    esp_efuse_mac_get_default(base_mac);
    Serial.printf("Base MAC (eFuse): %02X:%02X:%02X:%02X:%02X:%02X\n",
                  base_mac[0], base_mac[1], base_mac[2],
                  base_mac[3], base_mac[4], base_mac[5]);
}

void loop() {}