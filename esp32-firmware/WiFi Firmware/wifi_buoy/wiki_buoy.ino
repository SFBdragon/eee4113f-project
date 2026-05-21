/*
  Buoy side Firmware
*/

#include <WiFi.h>
#include <esp_now.h>

#define LED_PIN 2          // onboard LED (GPIO2 on classic ESP32 dev boards)
#define TX2_PIN 17         // Serial2 TX
#define RX2_PIN 16         // Serial2 RX

// SLIP special bytes (RFC 1055)
#define SLIP_END     0xC0   // frame delimiter
#define SLIP_ESC     0xDB   // escape
#define SLIP_ESC_END 0xDC   // escaped 0xC0
#define SLIP_ESC_ESC 0xDD   // escaped 0xDB

unsigned long ledOnTime = 0;     // when the LED was last turned on (0 = off)
const unsigned long LED_FLASH_MS = 20;

// MAC address of the SHIP-side ESP32 (the peer we send to)
uint8_t peerAddress[] = { 0xE0, 0x8C, 0xFE, 0x5C, 0x2A, 0x60 };

esp_now_peer_info_t peerInfo;

// Encode `data` as a SLIP frame and write it to the given stream
void slipEncodeAndWrite(Stream &out, const uint8_t *data, int len) {
  out.write(SLIP_END);   // start-of-frame delimiter
  for (int i = 0; i < len; i++) {
    switch (data[i]) {
      case SLIP_END:
        out.write(SLIP_ESC);
        out.write(SLIP_ESC_END);
        break;
      case SLIP_ESC:
        out.write(SLIP_ESC);
        out.write(SLIP_ESC_ESC);
        break;
      default:
        out.write(data[i]);
    }
  }
  out.write(SLIP_END);   // end-of-frame delimiter
}

void updateLed() {
  if (ledOnTime != 0 && (millis() - ledOnTime >= LED_FLASH_MS)) {
    digitalWrite(LED_PIN, LOW);
    ledOnTime = 0;
  }
}

void flashLed() {
  digitalWrite(LED_PIN, HIGH);
  ledOnTime = millis();
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  slipEncodeAndWrite(Serial2, data, len);
  flashLed();              // non-blocking: just sets the pin + timestamp
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(1000);

  WiFi.mode(WIFI_STA);
  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Buoy-side ready");
}

void loop() {
  static uint8_t buf[250];      // decoded frame, ESP-NOW max payload
  static size_t idx = 0;
  static bool escaped = false;  // did we just see SLIP_ESC?

  while (Serial2.available()) {
    uint8_t c = Serial2.read();

    if (escaped) {
      // Previous byte was ESC: translate this one
      if (c == SLIP_ESC_END)      { if (idx < sizeof(buf)) buf[idx++] = SLIP_END; }
      else if (c == SLIP_ESC_ESC) { if (idx < sizeof(buf)) buf[idx++] = SLIP_ESC; }
      // (a malformed escape sequence is just dropped)
      escaped = false;
    }
    else if (c == SLIP_END) {
      // End of frame: send whatever we've decoded, if anything
      if (idx > 0) {
        esp_now_send(peerAddress, buf, idx);
        idx = 0;
      }
      // A leading END (idx == 0) is just an empty frame / keepalive: ignore
    }
    else if (c == SLIP_ESC) {
      escaped = true;
    }
    else {
      if (idx < sizeof(buf)) buf[idx++] = c;
    }
  }

  updateLed();   // turn LED off once the flash interval has elapsed
}