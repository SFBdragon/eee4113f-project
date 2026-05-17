// connections.h
//
// Tamryn will write code files to implement this interface for Shaun to call.

#pragma once

#include <stdint.h>
#include <stdbool.h>


// --------------- Types --------------- //

typedef int32_t Status;
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
// I'm planning to use variable length mode as all my packets are small but
// vary considerably in size.
#define MAX_LORA_RECV_PACKET_LEN 255
#define MAX_LORA_SEND_PACKET_LEN 255

// WiFi payload limits. The UART framing protocol allocates 512 bytes per
// frame; the SEND frame format is [6-byte destmac][actual payload], so
// 506 is the largest payload that fits in one frame without changing the
// protocol (see wifi_protocol.h: WIFI_MAX_PAYLOAD = 512).
#define MAX_WIFI_RECV_PACKET_LEN 506
#define MAX_WIFI_SEND_PACKET_LEN 506

#define STATUS_SUCCESS 0
#define STATUS_RECEIVE_TIMEOUT -3
#define STATUS_MODULE_DETACHED -5
// add more statuses as necessary


// --------------- LoRa --------------- //

// Returns whether the LoRa module is visible as a device plugged into the computer.
bool is_lora_module_attached();

// This is called to perform all the LoRa module setup.
// Once it returns, the LoRa module should be listening for packets, if successful.
Status initialize_lora_module();

// This is called to perform all the LoRa module shutdown.
// Once it returns, send/recv will no longer be called until initialize is called again.
void shutdown_lora_module();

// This is used to help estimate the RTT for automatic transmission and control.
// This is directly related to Bandwidth, Spreading Factor, and Code Rate.
double get_lora_byterate();

// Read the memory from [data, data + len) bytes and send it as the data payload for a LoRa packet.
// This must be sent as one LoRa packet. `len` will be `MAX_LORA_SEND_PACKET_LEN` at most.
//
// This will not be called while `recv` is being called, and `recv` won't be called until this completes.
// Essentially, Shaun provides media access control for LoRa.
Status send_lora_packet(uint8_t *data, BufLen len);

// Block until a LoRa packet data payload is received.
// Writes the payload bytes into `data` and the payload length into `len`.
//
// This is not called while `send_lora_packet` is executing.
//
// `data` and `len` must describe one entire packet or nothing once this function returns.
// Feel free to overwrite `data` partially, fully, or not at all regardless of `len`.
Status recv_lora_packet(uint8_t data[MAX_LORA_RECV_PACKET_LEN], BufLen *len, uint32_t timeout_ms);

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
