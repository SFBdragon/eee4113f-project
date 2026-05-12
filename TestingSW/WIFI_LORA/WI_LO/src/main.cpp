#include <Arduino.h>

/**
 * PIN CONFIGURATION
 * ESP32-S3 pins 40 and 41. 
 * Connect: 
 * ESP TX (40) -> Ra-07H RX
 * ESP RX (41) -> Ra-07H TX
 */
#define TX_PIN 40
#define RX_PIN 41
#define STATUS_LED 2 

// Timing Constants
const unsigned long INTERVAL = 5000; // Increased to 5s for LoRa stability
unsigned long previousMillis = 0;
bool ledState = LOW;

void setup() {
  // Debug Serial (To Computer)
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to open

  // LoRa Serial (To Ra-07H)
  // HardwareSerial 1 on ESP32-S3
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Initialize LED Pin
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, ledState);

  Serial.println("--- Ra-07H LoRa Pass-through Initialized ---");
  Serial.printf("Settings: TX=%d, RX=%d, LED=%d\n", TX_PIN, RX_PIN, STATUS_LED);
  Serial.println("Type AT commands in the monitor to test.");
  Serial.println("-------------------------------------------");
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. HEARTBEAT / AUTOMATIC POLLING
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;

    // Toggle LED
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState);
    
    // Check if module is alive
    Serial1.print("AT\r\n");
    
    Serial.print("[System] Heartbeat Sent | LED: ");
    Serial.println(ledState ? "ON" : "OFF");
  }

  // 2. RECEIVE FROM Ra-07H
  // This logic captures full strings by waiting slightly for the UART buffer
  if (Serial1.available()) {
    Serial.print("[Ra-07H]: ");
    while (Serial1.available()) {
      char c = Serial1.read();
      Serial.print(c);
      // Tiny delay prevents the loop from ending mid-transmission 
      // due to slow UART speeds
      delay(2); 
    }
  }

  // 3. MANUAL PASS-THROUGH (Computer -> Ra-07H)
  if (Serial.available()) {
    // Read user input until newline
    String manualCmd = Serial.readStringUntil('\n');
    manualCmd.trim(); 
    
    if (manualCmd.length() > 0) {
      // Send to Ra-07H with required CRLF
      Serial1.print(manualCmd + "\r\n");
      Serial.print("[User]: ");
      Serial.println(manualCmd);
    }
  }
}