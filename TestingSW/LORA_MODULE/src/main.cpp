#include <esp_now.h>
#include <WiFi.h>

uint8_t buoyMAC[] = {0x08, 0x92, 0x72, 0x85, 0x8B, 0xF0};

typedef struct Message {
  uint8_t data[250];
  int len;
} Message;

Message msg;

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  // optional debug
}

void onReceive(const uint8_t *mac, const uint8_t *inData, int len) {
  // ESP-NOW → USB Serial
  Message received;
  memcpy(&received, inData, sizeof(received));
  Serial.write(received.data, received.len);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, buoyMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  // USB Serial → ESP-NOW
  if (Serial.available()) {
    delay(5); // let buffer fill a little
    msg.len = 0;
    while (Serial.available() && msg.len < 250) {
      msg.data[msg.len++] = Serial.read();
    }
    esp_now_send(buoyMAC, (uint8_t *)&msg, sizeof(msg));
  }
}