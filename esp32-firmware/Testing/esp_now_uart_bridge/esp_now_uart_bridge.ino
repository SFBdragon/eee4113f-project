/* =====================================================================
 *  ESP-NOW <-> UART transparent SLIP-framed bridge
 * ---------------------------------------------------------------------
 *  One firmware, two ends. Reads SLIP-encoded packets off a UART,
 *  forwards each complete frame over ESP-NOW to the peer, and writes
 *  received frames back out the UART verbatim.
 *
 *  The ESP32 is a DUMB PIPE: it never decodes the SLIP payload. It uses
 *  the SLIP END byte (0xC0) only to find packet boundaries. The SLIP
 *  codec stays in the STM32 / laptop code where it already works.
 *
 *  ---------------------------------------------------------------------
 *  SETUP — only three things to touch, all in the CONFIG block below:
 *    1. THIS_DEVICE   -> BUOY on one board, SHIP on the other
 *    2. PEER MAC addrs -> fill in the real MAC of each board
 *    3. UART pins/baud -> match Shaun & Glen's existing config
 *  ---------------------------------------------------------------------
 *  To find a board's MAC: flash this sketch, open Serial Monitor at
 *  115200, and it prints "My MAC address: ..." on boot. Copy that into
 *  the peer entry on the OTHER board.
 * ===================================================================== */

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

/* ====================== CONFIG — EDIT THIS BLOCK ====================== */

// --- 1. Which board is this? Set BUOY on one, SHIP on the other. -----
#define BUOY 0
#define SHIP 1
#define THIS_DEVICE  BUOY        // <-- CHANGE PER BOARD

// --- 2. Peer MAC addresses -------------------------------------------
// Fill these in with the real MACs (printed on boot, see header note).
// BUOY talks to SHIP's MAC; SHIP talks to BUOY's MAC.
static uint8_t MAC_BUOY[6] = { 0xD4, 0xE9, 0xF4, 0xE8, 0x7A, 0x68 };
static uint8_t MAC_SHIP[6] = { 0xE0, 0x8C, 0xFE, 0x5C, 0x2A, 0x60 };

#if THIS_DEVICE == BUOY
  static uint8_t *PEER_MAC = MAC_SHIP;     // buoy sends to ship
#else
  static uint8_t *PEER_MAC = MAC_BUOY;     // ship sends to buoy
#endif

// --- 3. Data UART (to STM32 or laptop) -------------------------------
//
// DATA_ON_USB selects WHERE this board reads/writes its data stream:
//
//   0 = data on UART1 (GPIO16/17), logs on USB/UART0  [DEFAULT, symmetric]
//       Use this with an external USB-UART adapter (CP2102 etc.) on
//       GPIO16/17. USB stays free for live [TX] ok=/fail= logs.
//
//   1 = data on USB/UART0 directly, NO external adapter, NO logs.
//       Use this to feed data straight through the ESP32's own USB port.
//       Handy when two identical CP2102s collide on macOS (same USB
//       serial number) — set DATA_ON_USB=1 on the SHIP board and drop
//       its CP2102 entirely, leaving only ONE CP2102 in the system.
//
// IMPORTANT: when DATA_ON_USB=1, logging MUST be off, because data and
// log text would otherwise share the same USB wire and corrupt each
// other (and the peer would forward the log spew into your data stream).
#define DATA_ON_USB     1           // <-- set to 1 on the ship for USB-fed data

#define DATA_UART_NUM   1           // used only when DATA_ON_USB == 0
#define DATA_BAUD       115200      // <-- MATCH existing code / test --baud
#define DATA_RX_PIN     16          // ESP32 receives data on this pin
#define DATA_TX_PIN     17          // ESP32 sends data out on this pin

// --- ESP-NOW radio channel (must match on both boards) ---------------
#define ESPNOW_CHANNEL  1

// --- Logging: set to 0 to silence the USB-serial link-health logs ----
// Forced OFF automatically when data is on USB (see note above).
#if DATA_ON_USB
  #define ENABLE_LOG    0
#else
  #define ENABLE_LOG    1
#endif

/* ==================== END CONFIG — code below ======================== */

// SLIP special bytes (RFC 1055). We only care about END as a boundary.
static const uint8_t SLIP_END = 0xC0;

// ESP-NOW hard limit. A SLIP frame must fit in one packet (confirmed
// <250B by design). Frames larger than this are dropped with a log.
static const size_t ESPNOW_MAX = 250;

// DataSerial is the data stream. On USB builds it's the native USB
// serial (Serial); otherwise it's UART1 on the configured pins.
#if DATA_ON_USB
  #define DataSerial Serial
#else
  HardwareSerial DataSerial(DATA_UART_NUM);
#endif

// Assembly buffer for the incoming UART -> radio direction.
static uint8_t  txBuf[ESPNOW_MAX];
static size_t   txLen = 0;

// Simple counters for link-health logging.
static uint32_t sentOK = 0, sentFail = 0, rxCount = 0, oversize = 0;

// ---------------------------------------------------------------------
static void logMac(const char *label, const uint8_t *mac) {
#if ENABLE_LOG
  Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n", label,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
}

// Callback after each unicast send: tells us whether the peer ACKed.
// Arduino-ESP32 core 3.3.x changed arg 1 from (const uint8_t *mac) to
// (const wifi_tx_info_t *). We don't use the arg, so we just match it.
static void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) sentOK++; else sentFail++;
#if ENABLE_LOG
  // Throttle: only print on failure or every 100th success, so the log
  // doesn't drown out anything useful at high packet rates.
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("[TX] FAIL  (ok=%lu fail=%lu)\n", sentOK, sentFail);
  } else if ((sentOK % 100) == 0) {
    Serial.printf("[TX] ok=%lu fail=%lu\n", sentOK, sentFail);
  }
#endif
}

// Callback when a packet arrives from the peer: dump it straight to UART.
// Signature matches Arduino-ESP32 core v3.x (esp_now_recv_info).
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len > 0) {
    DataSerial.write(data, (size_t)len);
    rxCount++;
#if ENABLE_LOG
    Serial.printf("[RX] %d bytes\n", len);   // <-- add this
#endif
  }
}

// Send one assembled frame over the air, then reset the buffer.
static void flushFrame() {
  if (txLen == 0) return;
  esp_err_t r = esp_now_send(PEER_MAC, txBuf, txLen);
#if ENABLE_LOG
  if (r != ESP_OK) Serial.printf("[TX] esp_now_send err=%d\n", r);
#endif
  txLen = 0;
}

void setup() {
#if ENABLE_LOG
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP-NOW UART bridge ===");
  Serial.println(THIS_DEVICE == BUOY ? "Role: BUOY" : "Role: SHIP");
#endif

  // Data UART to the host (STM32 / laptop).
#if DATA_ON_USB
  // Data flows over the native USB serial. No pin args; no logging
  // shares this wire. Pin defines above are unused in this build.
  DataSerial.begin(DATA_BAUD);
  delay(300);
#else
  DataSerial.begin(DATA_BAUD, SERIAL_8N1, DATA_RX_PIN, DATA_TX_PIN);
#endif

  // Bring up WiFi in station mode but never associate — ESP-NOW only.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

#if ENABLE_LOG
  Serial.print("My MAC address: ");
  Serial.println(WiFi.macAddress());   // <-- copy this into the peer board
#endif

  if (esp_now_init() != ESP_OK) {
#if ENABLE_LOG
    Serial.println("ESP-NOW init FAILED — halting.");
#endif
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  // Register the peer.
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;            // keep it simple; add a PMK later if needed
  if (esp_now_add_peer(&peer) != ESP_OK) {
#if ENABLE_LOG
    Serial.println("Failed to add peer — check PEER_MAC.");
#endif
  } else {
    logMac("Peer MAC:       ", PEER_MAC);
  }
}

void loop() {
  // Drain everything waiting on the UART each pass. We accumulate bytes
  // into txBuf and treat SLIP END (0xC0) as the frame boundary.
  while (DataSerial.available()) {
    uint8_t b = (uint8_t)DataSerial.read();

    if (txLen < ESPNOW_MAX) {
      txBuf[txLen++] = b;
    } else {
      // Frame exceeded the ESP-NOW limit before an END arrived. This
      // shouldn't happen given the <250B design guarantee; drop and
      // resync at the next END so one bad frame can't wedge the link.
      oversize++;
#if ENABLE_LOG
      Serial.printf("[RX-UART] oversize frame dropped (count=%lu)\n",
                    oversize);
#endif
      txLen = 0;
      // fall through: still need to honour this byte if it's an END
    }

    // SLIP END marks the end of a packet -> ship it.
    // We keep the END byte in the frame so the far side receives an
    // identical, well-formed SLIP packet (pure pass-through).
    if (b == SLIP_END && txLen > 0) {
      flushFrame();
    }
  }
}
