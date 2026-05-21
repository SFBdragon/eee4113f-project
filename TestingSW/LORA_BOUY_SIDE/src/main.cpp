#include <esp_now.h>
#include <WiFi.h>

uint8_t shipMAC[] = {0x08, 0x92, 0x72, 0x85, 0x0B, 0xB8};



typedef struct Message {
  uint8_t data[250];
  int len;
} Message;

Message msg;

void onReceive(const uint8_t *mac, const uint8_t *inData, int len) {
  Message received;
  memcpy(&received, inData, sizeof(received));
  Serial1.write(received.data, received.len);
}

void setup() {
  Serial.begin(115200);              // USB debug
  Serial1.begin(115200, SERIAL_8N1, 20, 21);  
  
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, shipMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  //delay(5000);
}

void loop() {
  Serial.println(WiFi.macAddress());
  if (Serial1.available()) {
    delay(5);
    msg.len = 0;
    while (Serial1.available() && msg.len < 250) {
      msg.data[msg.len++] = Serial1.read();
    }
    esp_now_send(shipMAC, (uint8_t *)&msg, sizeof(msg));
  }
}