// buoy_wifi.ino
//
// EEE4113F project — Buoy-side Wi-Fi firmware.
// Runs on the ESP32 dev board (Board B).
//
// Architecture:
//   - Wi-Fi: STA mode, connects to "EEE4113F_buoy_net"
//   - UDP: sends datagrams to ship at 192.168.4.1:4242,
//          listens on the same port for replies (--mode echo).
//   - UART: 921600 baud, same binary framing as ship_wifi.
//
// The buoy stays in waiting mode after boot; it only attempts the Wi-Fi
// association when the host sends a MSG_INIT frame.
//
// UDP rationale: see ship_wifi.ino header. Lossy but doesn't desync the
// UART framing on partial writes the way TCP did.

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_mac.h"

// ============================================================
// Configuration
// ============================================================

static const char *AP_SSID      = "EEE4113F_buoy_net";
static const char *AP_PASSWORD  = "eee4113f2025";
static const char *SHIP_IP      = "192.168.4.1";
static const uint16_t UDP_PORT  = 4242;
static const uint32_t UART_BAUD = 921600;

#define FRAME_SYNC0 0xAA
#define FRAME_SYNC1 0x55
#define MAX_PAYLOAD 512

#define MSG_SEND     0x01
#define MSG_STATUS_Q 0x02
#define MSG_SHUTDOWN 0x03
#define MSG_INIT     0x04

#define MSG_ACK    0x81
#define MSG_NACK   0x82
#define MSG_RECV   0x83
#define MSG_STATUS 0x84
#define MSG_READY  0x85
#define MSG_LOG    0x86

#define NACK_NO_PEER      0x01
#define NACK_TOO_LARGE    0x02
#define NACK_BAD_CRC      0x03
#define NACK_BAD_FORMAT   0x04
#define NACK_NOT_READY    0x05

WiFiUDP udp;
bool wifiConnected = false;
bool linkReady = false;   // true once Wi-Fi is up and UDP socket is bound

uint8_t rxBuf[MAX_PAYLOAD + 16];
size_t rxLen = 0;
enum RxState { RX_SYNC0, RX_SYNC1, RX_TYPE, RX_LEN_HI, RX_LEN_LO, RX_PAYLOAD, RX_CRC_HI, RX_CRC_LO };
RxState rxState = RX_SYNC0;
uint8_t curType;
uint16_t curLen;
uint16_t curCrcRx;
size_t curPayloadIdx;

uint8_t self_mac[6];

// Resolved ship address (parsed once after Wi-Fi associates).
IPAddress shipAddr;

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

void sendAck()  { sendFrame(MSG_ACK, nullptr, 0); }
void sendNack(uint8_t r) { sendFrame(MSG_NACK, &r, 1); }
void sendLog(const char *s) { sendFrame(MSG_LOG, (const uint8_t*)s, strlen(s)); }
void sendReady() {
  uint8_t p[6]; memcpy(p, self_mac, 6);
  sendFrame(MSG_READY, p, 6);
}
void sendStatus() {
  uint8_t p[10];
  p[0] = linkReady ? 1 : 0;
  p[1] = wifiConnected ? 1 : 0;
  int rssi = wifiConnected ? WiFi.RSSI() : 0;
  p[2] = (uint8_t)((rssi >> 8) & 0xFF);
  p[3] = (uint8_t)(rssi & 0xFF);
  memcpy(&p[4], self_mac, 6);
  sendFrame(MSG_STATUS, p, 10);
}

bool tryConnect() {
  sendLog("connecting to AP");
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(100);
  }
  if (WiFi.status() != WL_CONNECTED) {
    sendLog("AP connect timeout");
    return false;
  }
  wifiConnected = true;
  sendLog("AP connected");

  // Diagnostic: log what IP the buoy got and who its gateway is. The
  // gateway IS the ship in this softAP setup. If shipAddr resolves to
  // a different IP, that's our bug.
  {
    IPAddress local = WiFi.localIP();
    IPAddress gw    = WiFi.gatewayIP();
    char buf[120];
    snprintf(buf, sizeof(buf),
             "buoy IP=%u.%u.%u.%u gateway=%u.%u.%u.%u target=%s",
             local[0], local[1], local[2], local[3],
             gw[0], gw[1], gw[2], gw[3],
             SHIP_IP);
    sendLog(buf);
  }

  // Resolve ship IP and bind the local UDP port.
  if (!shipAddr.fromString(SHIP_IP)) {
    sendLog("bad SHIP_IP literal");
    return false;
  }
  if (!udp.begin(UDP_PORT)) {
    sendLog("UDP begin FAILED");
    return false;
  }
  linkReady = true;
  sendLog("UDP socket bound");
  return true;
}

void doShutdown() {
  if (linkReady) udp.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
  linkReady = false;
}

// UDP send counters for diagnostic logging.
static uint32_t udpSendOk = 0;
static uint32_t udpSendFail = 0;
static uint32_t udpLastReportMs = 0;

void processFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
  switch (type) {
    case MSG_INIT: {
      bool ok = tryConnect();
      if (ok) sendAck();
      else    sendNack(NACK_NOT_READY);
      break;
    }
    case MSG_SEND: {
      if (!linkReady) { sendNack(NACK_NO_PEER); return; }
      if (len < 6) { sendNack(NACK_BAD_FORMAT); return; }
      // payload = [6-byte destmac][data]. destmac is ignored — UDP has
      // a single fixed peer (the ship).
      const uint8_t *data = payload + 6;
      uint16_t dlen = len - 6;

      // Send with up to 5 retries, 1ms apart. On the ESP32, repeated
      // udp.endPacket() failures mean lwip's pbuf pool is full because
      // the Wi-Fi MAC isn't draining buffers fast enough. delay(1) is
      // one FreeRTOS tick, which lets the tcpip_thread run and free
      // some pbufs. 5ms of retry budget is generous but bounded.
      bool ok = false;
      size_t w = 0;
      for (int attempt = 0; attempt < 5; attempt++) {
        udp.beginPacket(shipAddr, UDP_PORT);
        w = udp.write(data, dlen);
        ok = udp.endPacket();
        if (ok && w == dlen) break;
        delay(1);
      }

      if (ok && w == dlen) {
        udpSendOk++;
        sendAck();
      } else {
        udpSendFail++;
        sendNack(NACK_NO_PEER);
      }
      break;
    }
    case MSG_STATUS_Q:
      sendStatus();
      break;
    case MSG_SHUTDOWN:
      doShutdown();
      sendAck();
      break;
    default:
      sendNack(NACK_BAD_FORMAT);
      break;
  }
}

void resetRx() { rxState = RX_SYNC0; curPayloadIdx = 0; }

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
    case RX_TYPE: curType = b; rxState = RX_LEN_HI; break;
    case RX_LEN_HI: curLen = ((uint16_t)b) << 8; rxState = RX_LEN_LO; break;
    case RX_LEN_LO:
      curLen |= b;
      if (curLen > MAX_PAYLOAD) { sendNack(NACK_TOO_LARGE); resetRx(); break; }
      curPayloadIdx = 0;
      rxState = (curLen == 0) ? RX_CRC_HI : RX_PAYLOAD;
      break;
    case RX_PAYLOAD:
      rxBuf[curPayloadIdx++] = b;
      if (curPayloadIdx == curLen) rxState = RX_CRC_HI;
      break;
    case RX_CRC_HI: curCrcRx = ((uint16_t)b) << 8; rxState = RX_CRC_LO; break;
    case RX_CRC_LO: {
      curCrcRx |= b;
      uint8_t cb[3] = { curType, (uint8_t)(curLen >> 8), (uint8_t)(curLen & 0xFF) };
      uint16_t crc = crc16_ccitt(cb, 3);
      for (size_t i = 0; i < curLen; i++) {
        crc ^= ((uint16_t)rxBuf[i]) << 8;
        for (int j = 0; j < 8; j++) {
          if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
          else              crc = (crc << 1);
        }
      }
      if (crc == curCrcRx) processFrame(curType, rxBuf, curLen);
      else sendNack(NACK_BAD_CRC);
      resetRx();
      break;
    }
  }
}

uint8_t udpRecvBuf[MAX_PAYLOAD];

void serviceUdp() {
  if (!linkReady) return;
  // Drain ALL pending packets every loop iteration.
  for (int guard = 0; guard < 32; guard++) {
    int psize = udp.parsePacket();
    if (psize <= 0) break;

    if (psize > (int)sizeof(udpRecvBuf)) {
      udp.read(udpRecvBuf, sizeof(udpRecvBuf));
      sendLog("UDP packet truncated (too large)");
      continue;
    }
    int got = udp.read(udpRecvBuf, psize);
    if (got <= 0) continue;

    uint8_t out[MAX_PAYLOAD + 6];
    memset(out, 0, 6);
    memcpy(out + 6, udpRecvBuf, got);
    sendFrame(MSG_RECV, out, (uint16_t)(6 + got));
  }
}

void setup() {
  // Enlarge the UART RX buffer BEFORE Serial.begin. 32KB gives us
  // ~350ms of buffering at 921600 baud — plenty of slack if the main
  // loop pauses briefly for lwip/Wi-Fi processing.
  Serial.setRxBufferSize(32768);
  Serial.begin(UART_BAUD);
  delay(500);

  esp_read_mac(self_mac, ESP_MAC_WIFI_STA);

  // Start in waiting mode — Wi-Fi off until MSG_INIT
  WiFi.mode(WIFI_OFF);

  delay(200);
  sendReady();
  sendLog("buoy waiting for INIT (UDP mode)");
}

void loop() {
  while (Serial.available()) {
    rxByte((uint8_t)Serial.read());
  }
  serviceUdp();

  // Periodic UDP send counter dump. Tells us how many packets the
  // buoy *thinks* it sent successfully versus how many endPacket()
  // returned false on. Compare with the ship's recv counter to find
  // the layer that's eating packets.
  if (linkReady) {
    uint32_t now = millis();
    if (now - udpLastReportMs > 2000) {
      udpLastReportMs = now;
      char buf[100];
      snprintf(buf, sizeof(buf),
               "udp send ok=%lu fail=%lu wifi=%s rssi=%ld",
               (unsigned long)udpSendOk, (unsigned long)udpSendFail,
               WiFi.status() == WL_CONNECTED ? "up" : "DOWN",
               (long)WiFi.RSSI());
      sendLog(buf);
    }
  }
}
