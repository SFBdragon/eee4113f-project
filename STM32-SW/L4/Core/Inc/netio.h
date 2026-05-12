// netio.h
//
// Tamryn will write code files to implement parts of this interface.
// e.g. `lora_driver.c` and `wifi_driver.c`
// Shaun will wirete code files to implement parts of this interface.
// e.g. `protocol.c`

#pragma once

#include <stdint.h>


// --------------- Types --------------- //

// See `STATUS_*` constants for semantics.
typedef uint16_t Status;
// The type used to describe buffer lengths.
typedef uint16_t BufLen;

// --------------- Constants --------------- //

// These are not final. Alter these if necessary.
//
// The SX1278 datasheet says that
// Variable-length mode: 0-255 bytes
// Fixed-length mode: 0-2047 bytes
// Unlimited-length mode: 0-? bytes (disables a bunch of features though)
// https://www.alldatasheet.com/datasheet-pdf/download/800241/SEMTECH/SX1278.html
//
// Note that LoRa packets range from pings and status information to control
// messages, so a fixed-length is unlikely to be a good idea.
// I'm hoping that I can fit all the message lengths into less than 256 bytes
// or break up the messages across multiple packets.
#define MAX_LORA_RECV_PACKET_LEN 64
#define MAX_LORA_SEND_PACKET_LEN 64

// These are not final. Alter these if necessary.
#define MAX_WIFI_RECV_PACKET_LEN 250
#define MAX_WIFI_SEND_PACKET_LEN 250

#define STATUS_SUCCESS 0
// add more statusses as necessary

// --------------- LoRa --------------- //

// This is called to perform all the LoRa module setup.
// Once it returns, the LoRa module should be listening for packets,
// if successful.
//
// Tamryn defines this function. Shaun calls it.
Status initialize_lora();

// Read the memory from [data, data + len) bytes and broadcast it via LoRa.
//
// Tamryn defines this function. Shaun calls it.
Status send_lora_packet(uint8_t *data, BufLen len);

// Provide the least recent unread packet in memory [data, data + len).
//
// Shaun defines this function. Tamryn calls it.
// Shaun will avoid having Shaun's error conditions affect Tam's code,
// thus an error code is not returned here.
void recv_lora_packet(uint8_t *data, BufLen len);

// --------------- WiFi --------------- //

// This is called to perform all the WiFi module setup.
// Once it returns, the WiFi module and driver should be ready
// for power_up_wifi to be called.
Status initialize_wifi();

// Does this return immediately or does this block until power-up is done?
Status power_up_wifi();

// Does this return immediately or does this block until power-down is done?
Status power_down_wifi();

// Read the memory from [data, data + len) bytes and send it off.
//
// Tamryn defines this function. Shaun calls it.
short send_wifi_packet(uint64_t macdst, uint8_t *data, uint16_t len);

// Provide the least recent unread packet in memory [data, data + len).
//
// Shaun defines this function. Tamryn calls it.
// Shaun will avoid having Shaun's error conditions affect Tam's code,
// thus an error code is not returned here.
void recv_wifi_packet(uint64_t *macsrc, uint8_t *data, uint16_t len);
