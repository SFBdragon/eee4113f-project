#include <Arduino.h>

#define TRIGGER_PIN   5
#define ONBOARD_LED   8
#define BAUD_RATE     115200

#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20

#define PAYLOAD_SIZE 10

uint8_t txBuffer[PAYLOAD_SIZE];

uint32_t packetNumber = 0;

void setup() {

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);

  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(BAUD_RATE);

  Serial1.begin(
    BAUD_RATE,
    SERIAL_8N1,
    SERIAL_RX_PIN,
    SERIAL_TX_PIN
  );

  Serial.println("UART TX/RX RAW MONITOR");
}

void buildPacket() {

  // Fill payload with changing pattern
  for (int i = 0; i < PAYLOAD_SIZE; i++) {
    txBuffer[i] = 'A' + (packetNumber % 26);
  }

  // Put packet number into first 4 bytes
  memcpy(txBuffer, &packetNumber, sizeof(packetNumber));
}

void sendPacket() {

  uint8_t startFrame[] = {0xAA, 0xAA, 0xAA, 0xAA};
  uint8_t endFrame[]   = {0xFF, 0xFF, 0x0D, 0x0A};

  digitalWrite(TRIGGER_PIN, HIGH);
  digitalWrite(ONBOARD_LED, HIGH);

  delay(10);

  // Send framing + payload
  Serial1.write(startFrame, sizeof(startFrame));
  Serial1.write(txBuffer, PAYLOAD_SIZE);
  Serial1.write(endFrame, sizeof(endFrame));

  Serial1.flush();

  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.print("Sent packet #: ");
  Serial.println(packetNumber);
}

void printRawSerial() {

  while (Serial1.available()) {

    uint8_t b = Serial1.read();

    // HEX print
    if (b < 0x10) {
      Serial.print("0");
    }

    Serial.print(b, HEX);
    Serial.print(" ");

    // ASCII print
    Serial.print("[");

    if (b >= 32 && b <= 126) {
      Serial.print((char)b);
    } else {
      Serial.print(".");
    }

    Serial.print("] ");

    Serial.println();
  }
}

void loop() {

  // Print ALL received raw UART bytes
  printRawSerial();

  // Send packet every 2 seconds
  static uint32_t lastSend = 0;

  if (millis() - lastSend >= 2000) {

    buildPacket();

    sendPacket();

    packetNumber++;

    lastSend = millis();
  }
}