
#include <stdint.h>
#include "../include/drivers.h"

// --------------- LoRa --------------- //

// Returns whether the LoRa module is visible as a device plugged into the computer.
bool is_lora_module_attached() {
    return true;
}

// This is called to perform all the LoRa module setup.
// Once it returns, the LoRa module should be listening for packets, if successful.
Status initialize_lora_module() {
    return STATUS_SUCCESS;
}

// This is called to perform all the LoRa module shutdown.
// Once it returns, send/recv will no longer be called until initialize is called again.
void shutdown_lora_module() {

}

// Read the memory from [data, data + len) bytes and send it as the data payload for a LoRa packet.
Status send_lora_packet(uint8_t *data, BufLen len) {
    return STATUS_SUCCESS;
}

// Block until a LoRa packet data payload is received.
// Writes the payload bytes into `data` and the payload length into `len`.
Status recv_lora_packet(uint8_t data[MAX_LORA_RECV_PACKET_LEN], BufLen *len, uint32_t timeout_ms) {
    while (1) { }
}

// --------------- WiFi --------------- //

// Returns whether the WiFi module is visible as a device plugged into the computer.
bool is_wifi_module_attached() {
    return true;
}

// This is called to perform all the WiFi module setup.
// Once it returns, send/recv may be called.
Status initialize_wifi_module() {
    return STATUS_SUCCESS;
}

// This is called to perform all the WiFi module shutdown.
// Once it returns, send/recv will no longer be called until initialize is called again.
void shutdown_wifi_module() {

}

// Read the memory from [data, data + len) bytes and send it as the data payload for a WiFi packet.
// `destmac` specified the target MAC address. This may be FF:FF:FF:FF:FF:FF (broadcast) or a device MAC.
Status send_wifi_packet(uint64_t destmac, uint8_t *data, BufLen len) {
    return STATUS_SUCCESS;
}

// Writes the payload bytes into `data` and the payload length into `len`.
// Writes the sender's MAC into the low 48 bits of `srcmac`.
Status recv_wifi_packet(uint64_t *srcmac, uint8_t data[MAX_WIFI_RECV_PACKET_LEN], BufLen *len) {
    while (1) { }
}
