/*
 * Serial <-> Serial1 bridge for ESP32
 *
 * Serial  : USB-UART (via built-in USB-to-serial or USB OTG)
 * Serial1 : Hardware UART1, explicitly mapped to RX=20, TX=21
 *
 * Adjust BAUD_SERIAL1 independently if the target device runs
 * at a different rate than the host side.
 */

#include <Arduino.h>

#define PIN_RX1       20
#define PIN_TX1       21
#define BAUD_SERIAL   115200
#define BAUD_SERIAL1  115200


#define LED_PIN 2

void setup() {
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(BAUD_SERIAL);
    Serial1.begin(BAUD_SERIAL1, SERIAL_8N1, PIN_RX1, PIN_TX1);
}

void loop() {
    // Serial -> Serial1
    while (Serial.available()) {
        digitalWrite(LED_PIN, 1);
        Serial1.write(Serial.read());
    }

    // Serial1 -> Serial
    while (Serial1.available()) {
        digitalWrite(LED_PIN, 1);
        Serial.write(Serial1.read());
    }
}
