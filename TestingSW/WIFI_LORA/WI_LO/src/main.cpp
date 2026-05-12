#include <Arduino.h>

#define TX_PIN 40
#define RX_PIN 41

void setup() {
  // Debug output to PC
  Serial.begin(115200);
  
  // Communication to Ra-07H
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial.println("Sending 'A' every second...");
}

void loop() {
  // Send the literal character 'A' (ASCII 65)
  Serial1.print('A'); 
  
  // Also print to your PC so you know it's happening
  Serial.print("Sent: A | ");

  // Read back anything the module sends
  if (Serial1.available()) {
    Serial.print("Response: ");
    while (Serial1.available()) {
      Serial.write(Serial1.read());
    }
  }
  Serial.println();

  delay(1000); 
}