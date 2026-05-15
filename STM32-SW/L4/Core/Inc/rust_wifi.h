#pragma once

#include <stdint.h>
#include <stdbool.h>

// --- Types ---

typedef uint16_t LoRaAddr;

typedef uint16_t (*CrcFn)(const uint8_t* data, uintptr_t len);

typedef void (*WifiSetTimerFn)(uint32_t delay_ms);
typedef void (*WifiCancelTimerFn)(void);
typedef uint32_t (*WifiGetTimeFn)(void);

// --- WiFi Ping Functions ---

#define WIFI_PING_SIZE 7

/**
* Writes the bytes of a ping to a buffer.
* Assumes the buffer length is at least wifi_ping_len().
*/
void wifi_ping_write(LoRaAddr addr, CrcFn crc_fn, uint8_t* buf);

// --- WiFi Sender Functions ---

/**
* Initializes the WiFi sender and attempts a connection.
* Returns true if wifi_send_next should be called.
*/
bool wifi_connect(
    LoRaAddr controller_addr,
    WifiSetTimerFn set_timer,
    WifiCancelTimerFn cancel_timer,
    WifiGetTimeFn get_time,
    CrcFn crc_fn
);

/**
* Prepares the next packet to be sent.
* Argument: mac (output) - the destination MAC address.
* Argument: data (output) - pointer to the start of the payload buffer.
* Argument: len (output) - the length of the payload.
* Returns true if a packet was prepared and should be sent.
*/
bool wifi_send_next(uint64_t* mac, const uint8_t** data, uint16_t* len);

/**
* Processes a received ACK.
* Returns true if wifi_send_next should be called.
*/
bool wifi_on_recv_ack(uint64_t mac, const uint8_t* data, uint16_t len);

/**
* Handles a timer timeout event.
* Returns true if wifi_send_next should be called.
*/
bool wifi_on_timeout(void);

/**
* Queues a message for transmission.
* Returns true if wifi_send_next should be called.
*/
bool wifi_push_message(const uint8_t* data, uint16_t len);

/**
* Returns how many bytes are currently available in the outbound buffer.
*/
uint32_t wifi_available_payload_bytes(void);
