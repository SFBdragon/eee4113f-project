/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 2          // onboard LED (GPIO2 on classic ESP32 dev boards)
#define TX2_PIN 17         // Serial2 TX
#define RX2_PIN 16         // Serial2 RX

#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

unsigned long ledOnTime = 0;     // when the LED was last turned on (0 = off)
const unsigned long LED_FLASH_MS = 20;

// REPLACE WITH YOUR RECEIVER MAC Address
static uint8_t MAC_BUOY[6] = { 0xD4, 0xE9, 0xF4, 0xE8, 0x7A, 0x68 };
static uint8_t MAC_SHIP[6] = { 0xE0, 0x8C, 0xFE, 0x5C, 0x2A, 0x60 };

//uint8_t broadcastAddress[] = { 0xD4, 0xE9, 0xF4, 0xE8, 0x7A, 0x68 };

// MAC address of the BUOY-side ESP32 (the peer we send to)
uint8_t peerAddress[] = { 0xD4, 0xE9, 0xF4, 0xE8, 0x7A, 0x68 };

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

// Called when data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // Re-encode the payload as SLIP and send out the USB serial connection
  slipEncodeAndWrite(Serial, data, len);
  // Flash LED on each completed frame sent
  flashLed();   // turn LED off once the flash interval has elapsed
}

// Called after a send, reports success/failure
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
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

  // Register the peer
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;      // 0 = use current WiFi channel
  peerInfo.encrypt = false;  // keep it simple for now

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Ship-side ready");
}

void loop() {
  static uint8_t buf[250];
  static size_t idx = 0;
  static bool escaped = false;

  while (Serial.available()) {
    uint8_t c = Serial.read();

    if (escaped) {
      if (c == SLIP_ESC_END)      { if (idx < sizeof(buf)) buf[idx++] = SLIP_END; }
      else if (c == SLIP_ESC_ESC) { if (idx < sizeof(buf)) buf[idx++] = SLIP_ESC; }
      escaped = false;
    }
    else if (c == SLIP_END) {
      if (idx > 0) {
        esp_now_send(peerAddress, buf, idx);
        idx = 0;

        // Flash LED on each completed frame sent
        //digitalWrite(LED_PIN, HIGH);
        //delay(20);
        //digitalWrite(LED_PIN, LOW);
      }
    }
    else if (c == SLIP_ESC) {
      escaped = true;
    }
    else {
      if (idx < sizeof(buf)) buf[idx++] = c;
    }
  }

  updateLed();
}