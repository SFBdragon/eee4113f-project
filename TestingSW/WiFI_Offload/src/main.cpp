#include <Arduino.h>

#define ONBOARD_LED 8
#define BAUD_RATE 115200
#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20

// Must match SHARC Buoy emulator
#define DATA_SIZE 24576
#define BLOCK_SIZE 512
#define HEADER_BYTES 4 // 16-bit length + 16-bit CRC = 4 bytes

uint8_t receivedData[DATA_SIZE];
uint8_t expectedData[DATA_SIZE];
uint8_t blockBuffer[BLOCK_SIZE];
uint32_t bytesReceived = 0;

void setup() {
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(115200);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  // Rebuild expected dataset (same pattern as SHARC emulator)
  for (int i = 0; i < DATA_SIZE; i++) {
    expectedData[i] = (i % 26) + 65;
  }

  Serial.println("Wi-Fi Emulator Ready. Waiting for offload...");
}

void loop() {
  // Read incoming data block by block
  while (Serial1.available() && bytesReceived < DATA_SIZE) {
    // Read one full 512-byte block
    uint32_t blockBytes = 0;
    while (blockBytes < BLOCK_SIZE && Serial1.available()) {
      blockBuffer[blockBytes++] = Serial1.read();
    }

    // Skip 4-byte header (16-bit length + 16-bit CRC)
    if (blockBytes > HEADER_BYTES) {
      uint32_t payloadLen = blockBytes - HEADER_BYTES;
      memcpy(&receivedData[bytesReceived], 
             &blockBuffer[HEADER_BYTES], 
             payloadLen);
      bytesReceived += payloadLen;
    }
  }

  // Once full dataset received, verify integrity
  if (bytesReceived >= DATA_SIZE) {
    digitalWrite(ONBOARD_LED, HIGH);
    Serial.println("\n--- Offload Complete. Verifying integrity... ---");

    bool pass = true;
    uint32_t errorCount = 0;

    for (int i = 0; i < DATA_SIZE; i++) {
      if (receivedData[i] != expectedData[i]) {
        pass = false;
        errorCount++;
        if (errorCount <= 10) { // Print first 10 errors only
          Serial.printf("MISMATCH at byte %d: got 0x%02X expected 0x%02X\n",
                        i, receivedData[i], expectedData[i]);
        }
      }
    }

    if (pass) {
      Serial.println("PASS: Received data matches transmitted dataset exactly.");
    } else {
      Serial.printf("FAIL: %lu mismatched bytes out of %d.\n", 
                    errorCount, DATA_SIZE);
    }

    // Reset for next test
    bytesReceived = 0;
    memset(receivedData, 0, DATA_SIZE);
    digitalWrite(ONBOARD_LED, LOW);
    Serial.println("Reset. Waiting for next offload...");
  }
}