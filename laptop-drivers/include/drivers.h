// connections.h
//
// Tamryn will write code files to implement this interface for Shaun to call.

#pragma once

#include <cstdint>
#include <stdint.h>
#include <stdbool.h>


// --------------- Types --------------- //

typedef int32_t Status;
typedef uintptr_t BufLen;


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
#define STATUS_MODULE_DETACHED -5
// add more statusses as necessary


// --------------- LoRa --------------- //

// Returns whether the LoRa module is visible as a device plugged into the computer.
bool is_lora_module_attached();

// This is called to perform all the LoRa module setup.
// Once it returns, the LoRa module should be listening for packets, if successful.
Status initialize_lora_module();

// This is called to perform all the LoRa module shutdown.
// Once it returns, send/recv will no longer be called until initialize is called again.
void shutdown_lora_module();

// Read the memory from [data, data + len) bytes and send it as the data payload for a LoRa packet.
Status send_lora_packet(uint8_t *data, BufLen len);

// Block until a LoRa packet data payload is received.
// Writes the payload bytes into `data` and the payload length into `len`.
Status recv_lora_packet(uint8_t data[MAX_LORA_RECV_PACKET_LEN], BufLen *len);

// --------------- WiFi --------------- //

// Returns whether the WiFi module is visible as a device plugged into the computer.
bool is_wifi_module_attached();

// This is called to perform all the WiFi module setup.
// Once it returns, send/recv may be called.
Status initialize_wifi_module();

// This is called to perform all the WiFi module shutdown.
// Once it returns, send/recv will no longer be called until initialize is called again.
void shutdown_wifi_module();

// Read the memory from [data, data + len) bytes and send it as the data payload for a WiFi packet.
// `destmac` specified the target MAC address. This may be FF:FF:FF:FF:FF:FF (broadcast) or a device MAC.
Status send_wifi_packet(uint64_t destmac, uint8_t *data, BufLen len);

// Block until a WiFi packet data payload is received.
// Writes the payload bytes into `data` and the payload length into `len`.
// Writes the sender's MAC into the low 48 bits of `srcmac`.
Status recv_wifi_packet(uint64_t *srcmac, uint8_t data[MAX_WIFI_RECV_PACKET_LEN], BufLen *len);
