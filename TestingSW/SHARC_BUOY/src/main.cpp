#include <Arduino.h>

#define TRIGGER_PIN 5
#define ONBOARD_LED 8
#define BAUD_RATE 115200

// Hardware UART pins
#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20

uint8_t dataBuffer[120]; 
uint32_t packetCounter = 0; // Track which packet we are on

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(BAUD_RATE);
  // Use Serial1 for the physical TX/RX pins
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  // Fill buffer with a recognizable pattern
  for(int i = 0; i < 101; i++) {
    dataBuffer[i] = (i % 26) + 65; 
  }
}

void loop() {
  // 1. START SIGNAL & TRIGGER
  // Pull trigger HIGH before sending data to capture the whole frame
  digitalWrite(TRIGGER_PIN, HIGH);
  digitalWrite(ONBOARD_LED, HIGH); 
  delay(10); // Short lead-in delay for the analyzer to sync

  // 2. SEND START FRAME (4 bytes of 0xAA)
  uint8_t startFrame[] = {0xAA, 0xAA, 0xAA, 0xAA};
  Serial1.write(startFrame, 4);

  // 3. SEND UNIQUE IDENTIFIER
  // This sends the current packet number as 4 raw bytes
  Serial1.write((uint8_t*)&packetCounter, 4);

  // 4. SEND LARGE PAYLOAD
  Serial1.write(dataBuffer, 110);

  // 5. SEND END FRAME
  uint8_t endFrame[] = {0xFF, 0xFF, 0x0D, 0x0A};
  Serial1.write(endFrame, 4);

  // 6. STOP SIGNAL
  Serial1.flush(); // Ensure all bytes are physically off the wire
  delay(5);        // Tiny tail delay
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  // Increment the counter for the next packet
  packetCounter++;

  delay(200); // Wait 1 second between bursts
}