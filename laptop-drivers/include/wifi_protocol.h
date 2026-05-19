// wifi_protocol.h
//
// UART framing protocol between laptop driver and ESP32 firmware.
// Shared definitions; the .ino files duplicate these constants for
// portability with the Arduino build system.

#pragma once
#include <stdint.h>

#define WIFI_FRAME_SYNC0 0xAA
#define WIFI_FRAME_SYNC1 0x55

// Host → ESP32
#define WIFI_MSG_SEND     0x01
#define WIFI_MSG_STATUS_Q 0x02
#define WIFI_MSG_SHUTDOWN 0x03
#define WIFI_MSG_INIT     0x04

// ESP32 → Host
#define WIFI_MSG_ACK    0x81
#define WIFI_MSG_NACK   0x82
#define WIFI_MSG_RECV   0x83
#define WIFI_MSG_STATUS 0x84
#define WIFI_MSG_READY  0x85
#define WIFI_MSG_LOG    0x86

#define WIFI_NACK_NO_PEER     0x01
#define WIFI_NACK_TOO_LARGE   0x02
#define WIFI_NACK_BAD_CRC     0x03
#define WIFI_NACK_BAD_FORMAT  0x04
#define WIFI_NACK_NOT_READY   0x05

#define WIFI_MAX_PAYLOAD 512
#define WIFI_UART_BAUD 921600

// CRC-16/CCITT (poly 0x1021, init 0xFFFF, no reflect)
static inline uint16_t wifi_crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}