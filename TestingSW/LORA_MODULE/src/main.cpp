#include <Arduino.h>

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
}

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