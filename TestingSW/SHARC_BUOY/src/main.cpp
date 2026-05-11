#include <Arduino.h>

#define TRIGGER_PIN 5
#define ONBOARD_LED 8
#define BAUD_RATE 115200

char dataBuffer[512];
bool ledState = false;

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);

  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(BAUD_RATE);

  memset(dataBuffer, 'A', 511);
  dataBuffer[511] = '\0';
}

void loop() {
  // Set trigger to HIGH for interrupt
  digitalWrite(ONBOARD_LED, LOW); //ACTIVE LOW
  digitalWrite(TRIGGER_PIN, HIGH);
  delay(1000);                       // Delay to wait for interrupt

  // 3. Send 511 bytes of data (exclude null terminator)
  Serial.write((uint8_t*)dataBuffer, 511);

  digitalWrite(ONBOARD_LED, HIGH);
  digitalWrite(TRIGGER_PIN, LOW);
  // 4. Wait remainder of 5-second cycle before next toggle
  delay(1000);                       // ← THIS WAS MISSING entirely
}                                    // ← closing brace was missing