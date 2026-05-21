// ship_wifi.ino
//
// EEE4113F project — Ship-side Wi-Fi firmware.
// Runs on the ESP32 dev board (Board A).
//
// Architecture:
//   - Wi-Fi: Soft-AP, broadcasts SSID "EEE4113F_buoy_net"
//   - UDP: receiver on port 4242, accepts datagrams from any peer
//   - UART: 921600 baud, custom binary framing with the laptop driver
//
// Why UDP not TCP: at the throughput the ATP demands, TCP's
// per-segment ACK + window dynamics + buffer back-pressure consistently
// caused the buoy's TCP send to return short, which desynced the
// length-prefixed framing on this side. UDP datagrams preserve message
// boundaries natively (no length prefix needed) and the kernel/lwip
// drops on the floor when overloaded instead of corrupting the stream.
// Packet loss is the only failure mode — measured at the application
// layer as a difference between sent and received byte counts.

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_mac.h"

// ============================================================
// Configuration
// ============================================================

static const char *AP_SSID     = "EEE4113F_buoy_net";
static const char *AP_PASSWORD = "eee4113f2025";
static const int   AP_CHANNEL  = 6;
static const uint16_t UDP_PORT = 4242;
static const uint32_t UART_BAUD = 921600;

// ============================================================
// UART framing protocol
//   [0xAA] [0x55] [TYPE:1] [LEN_HI] [LEN_LO] [PAYLOAD...] [CRC_HI] [CRC_LO]
// CRC is CRC-16/CCITT over TYPE+LEN+PAYLOAD.
// ============================================================

#define FRAME_SYNC0 0xAA
#define FRAME_SYNC1 0x55
#define MAX_PAYLOAD 512

// Host → ESP32
#define MSG_SEND     0x01
#define MSG_STATUS_Q 0x02
#define MSG_SHUTDOWN 0x03
#define MSG_INIT     0x04

// ESP32 → Host
#define MSG_ACK    0x81
#define MSG_NACK   0x82
#define MSG_RECV   0x83
#define MSG_STATUS 0x84
#define MSG_READY  0x85
#define MSG_LOG    0x86

// NACK reasons
#define NACK_NO_PEER      0x01
#define NACK_TOO_LARGE    0x02
#define NACK_BAD_CRC      0x03
#define NACK_BAD_FORMAT   0x04
#define NACK_NOT_READY    0x05

// ============================================================
// State
// ============================================================

WiFiUDP udp;
bool wifiInitialized = false;

// Last peer we heard from over UDP. Used as the implicit reply target
// for MSG_SEND when destmac is broadcast. Updated every time we receive
// a datagram. Stays valid across packet boundaries.
IPAddress lastPeerIp;
uint16_t  lastPeerPort = 0;
bool      lastPeerKnown = false;

uint8_t rxBuf[MAX_PAYLOAD + 16];
size_t rxLen = 0;
enum RxState { RX_SYNC0, RX_SYNC1, RX_TYPE, RX_LEN_HI, RX_LEN_LO, RX_PAYLOAD, RX_CRC_HI, RX_CRC_LO };
RxState rxState = RX_SYNC0;
uint8_t curType;
uint16_t curLen;
uint16_t curCrcRx;
size_t curPayloadIdx;

uint8_t self_mac[6];

// ============================================================
// CRC-16/CCITT (poly 0x1021, init 0xFFFF, no reflect)
// ============================================================
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

// ============================================================
// Frame TX
// ============================================================

void sendFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t hdr[5];
  hdr[0] = FRAME_SYNC0;
  hdr[1] = FRAME_SYNC1;
  hdr[2] = type;
  hdr[3] = (len >> 8) & 0xFF;
  hdr[4] = len & 0xFF;

  uint16_t crc = 0xFFFF;
  uint8_t cb[3] = { type, hdr[3], hdr[4] };
  crc = crc16_ccitt(cb, 3);
  if (payload && len) {
    for (uint16_t i = 0; i < len; i++) {
      crc ^= ((uint16_t)payload[i]) << 8;
      for (int b = 0; b < 8; b++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else              crc = (crc << 1);
      }
    }
  }

  Serial.write(hdr, 5);
  if (payload && len) Serial.write(payload, len);
  uint8_t tail[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
  Serial.write(tail, 2);
}

void sendAck()   { sendFrame(MSG_ACK,  nullptr, 0); }
void sendNack(uint8_t reason) { sendFrame(MSG_NACK, &reason, 1); }
void sendLog(const char *msg) { sendFrame(MSG_LOG,  (const uint8_t*)msg, strlen(msg)); }

void sendStatus() {
  uint8_t payload[10];
  payload[0] = lastPeerKnown ? 1 : 0;
  payload[1] = WiFi.softAPgetStationNum();
  payload[2] = 0;
  payload[3] = 0;
  memcpy(&payload[4], self_mac, 6);
  sendFrame(MSG_STATUS, payload, 10);
}

void sendReady() {
  uint8_t payload[6];
  memcpy(payload, self_mac, 6);
  sendFrame(MSG_READY, payload, 6);
}

// ============================================================
// Frame RX state machine
// ============================================================

void resetRx() {
  rxState = RX_SYNC0;
  rxLen = 0;
  curPayloadIdx = 0;
}

void processFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
  switch (type) {
    case MSG_SEND: {
      // For UDP, "sending" means addressing a datagram to the last peer
      // we received from. If we've never received anything, NACK.
      if (!lastPeerKnown) { sendNack(NACK_NO_PEER); return; }
      if (len < 6) { sendNack(NACK_BAD_FORMAT); return; }
      // payload = [6-byte destmac][data]. destmac is ignored for UDP.
      const uint8_t *data = payload + 6;
      uint16_t dlen = len - 6;

      udp.beginPacket(lastPeerIp, lastPeerPort);
      size_t w = udp.write(data, dlen);
      bool ok = udp.endPacket();
      if (ok && w == dlen) sendAck();
      else sendNack(NACK_NO_PEER);
      break;
    }
    case MSG_STATUS_Q:
      sendStatus();
      break;
    case MSG_INIT:
      sendAck();
      break;
    case MSG_SHUTDOWN:
      sendAck();
      break;
    default:
      sendNack(NACK_BAD_FORMAT);
      break;
  }
}

void rxByte(uint8_t b) {
  switch (rxState) {
    case RX_SYNC0:
      if (b == FRAME_SYNC0) rxState = RX_SYNC1;
      break;
    case RX_SYNC1:
      if (b == FRAME_SYNC1) rxState = RX_TYPE;
      else if (b == FRAME_SYNC0) rxState = RX_SYNC1;
      else rxState = RX_SYNC0;
      break;
    case RX_TYPE:
      curType = b;
      rxState = RX_LEN_HI;
      break;
    case RX_LEN_HI:
      curLen = ((uint16_t)b) << 8;
      rxState = RX_LEN_LO;
      break;
    case RX_LEN_LO:
      curLen |= b;
      if (curLen > MAX_PAYLOAD) {
        sendNack(NACK_TOO_LARGE);
        resetRx();
        break;
      }
      curPayloadIdx = 0;
      if (curLen == 0) rxState = RX_CRC_HI;
      else rxState = RX_PAYLOAD;
      break;
    case RX_PAYLOAD:
      rxBuf[curPayloadIdx++] = b;
      if (curPayloadIdx == curLen) rxState = RX_CRC_HI;
      break;
    case RX_CRC_HI:
      curCrcRx = ((uint16_t)b) << 8;
      rxState = RX_CRC_LO;
      break;
    case RX_CRC_LO:
      curCrcRx |= b;
      {
        uint8_t cb[3] = { curType, (uint8_t)(curLen >> 8), (uint8_t)(curLen & 0xFF) };
        uint16_t crc = crc16_ccitt(cb, 3);
        for (size_t i = 0; i < curLen; i++) {
          crc ^= ((uint16_t)rxBuf[i]) << 8;
          for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = (crc << 1);
          }
        }
        if (crc == curCrcRx) {
          processFrame(curType, rxBuf, curLen);
        } else {
          sendNack(NACK_BAD_CRC);
        }
      }
      resetRx();
      break;
  }
}

// ============================================================
// UDP receive → forward to host
// ============================================================

uint8_t udpRecvBuf[MAX_PAYLOAD];
static uint32_t udpRecvCount = 0;
static uint32_t udpLastReportMs = 0;

void serviceUdp() {
  // Drain ALL pending packets every loop iteration. lwip's default UDP
  // recv queue is small (~6 packets); if we only handle one per loop
  // we lose packets at high rates.
  for (int guard = 0; guard < 32; guard++) {
    int psize = udp.parsePacket();
    if (psize <= 0) break;

    lastPeerIp   = udp.remoteIP();
    lastPeerPort = udp.remotePort();
    lastPeerKnown = true;
    udpRecvCount++;

    if (psize > (int)sizeof(udpRecvBuf)) {
      // Datagram bigger than our buffer; truncate-read and drop.
      udp.read(udpRecvBuf, sizeof(udpRecvBuf));
      sendLog("UDP packet truncated (too large)");
      continue;
    }

    int got = udp.read(udpRecvBuf, psize);
    if (got <= 0) continue;

    // Build RECV frame: [6 srcmac (zeros — UDP doesn't expose peer MAC)][data]
    uint8_t out[MAX_PAYLOAD + 6];
    memset(out, 0, 6);
    memcpy(out + 6, udpRecvBuf, got);
    sendFrame(MSG_RECV, out, (uint16_t)(6 + got));
  }

  // Periodic diagnostic: every 2 seconds, log how many UDP packets
  // we've seen total and from which peer. Tells us if the ship's
  // lwip stack is receiving anything at all.
  uint32_t now = millis();
  if (now - udpLastReportMs > 2000) {
    udpLastReportMs = now;
    char buf[100];
    if (lastPeerKnown) {
      snprintf(buf, sizeof(buf), "udp recv total=%lu last_peer=%u.%u.%u.%u:%u",
               (unsigned long)udpRecvCount,
               lastPeerIp[0], lastPeerIp[1], lastPeerIp[2], lastPeerIp[3],
               (unsigned)lastPeerPort);
    } else {
      snprintf(buf, sizeof(buf), "udp recv total=%lu (no peer ever)",
               (unsigned long)udpRecvCount);
    }
    sendLog(buf);
  }
}

// ============================================================
// Setup & loop
// ============================================================

void setup() {
  // Enlarge the UART buffers BEFORE Serial.begin.
  // RX: laptop sends commands; needs slack if loop() pauses.
  // TX: ship forwards every received UDP packet as MSG_RECV; if TX
  //     blocks on full FIFO, serviceUdp() can't drain UDP fast.
  Serial.setRxBufferSize(32768);
  Serial.setTxBufferSize(8192);
  Serial.begin(UART_BAUD);
  delay(500);

  esp_read_mac(self_mac, ESP_MAC_WIFI_STA);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, 1);
  delay(100);

  if (!udp.begin(UDP_PORT)) {
    sendLog("UDP begin FAILED");
  }

  wifiInitialized = true;

  delay(200);
  sendReady();
  sendLog("ship AP up (UDP mode)");

  // Diagnostic: log the actual IP the softAP is using and what port we
  // bound to. If the buoy is sending to a different address, we'll see
  // it here.
  {
    IPAddress ip = WiFi.softAPIP();
    char buf[80];
    snprintf(buf, sizeof(buf), "softAP IP=%u.%u.%u.%u port=%u",
             ip[0], ip[1], ip[2], ip[3], (unsigned)UDP_PORT);
    sendLog(buf);
  }
}

void loop() {
  while (Serial.available()) {
    rxByte((uint8_t)Serial.read());
  }
  serviceUdp();

  // FreeRTOS task yield. Without this, loop() (running on app_cpu)
  // hogs the core and starves the lwip tcpip_thread, which then can't
  // process incoming Wi-Fi frames into UDP datagrams. Symptom: ESP32
  // gets hot from the busy loop, UDP recv rate drops to near-zero.
  // delay(0) yields without blocking for any wall-clock time.
  delay(0);
}
