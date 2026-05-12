#include <Arduino.h>

#define TRIGGER_PIN 5
#define ONBOARD_LED 8
#define BAUD_RATE 115200

// Hardware UART pins
#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20

// 1KB Buffer to make the packet "long" on the wire
uint8_t dataBuffer[1024]; 

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  
  Serial.begin(BAUD_RATE);
  // Use Serial1 for the physical TX/RX pins
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  // Fill buffer with a recognizable pattern (ASCII 'A' through 'Z' repeating)
  for(int i = 0; i < 1024; i++) {
    dataBuffer[i] = (i % 26) + 65; 
  }

  digitalWrite(ONBOARD_LED, HIGH); // Off
}

void loop() {
  // 1. START SIGNAL

  digitalWrite(ONBOARD_LED, HIGH); // LED ON IS TRIGGER
  delay(100);
  // 2. SEND START FRAME (4 bytes of 0xAA for clear pulsing)
  uint8_t startFrame[] = {0xAA, 0xAA, 0xAA, 0xAA};
  Serial1.write(startFrame, 4);

  // 3. SEND LARGE PAYLOAD
  Serial1.write(dataBuffer, 1024);

  // 4. SEND END FRAME
  uint8_t endFrame[] = {0xFF, 0xFF, 0x0D, 0x0A};
  Serial1.write(endFrame, 4);

  // 5. STOP SIGNAL
  Serial1.flush();                   // ← ADD THIS: wait for TX to complete
  delay(100);                         // ← small gap before pulling low
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW); // LED OFF

  //Serial.println("Massive packet sent.");
  delay(500); // Wait 2 seconds to make the burst obvious
}