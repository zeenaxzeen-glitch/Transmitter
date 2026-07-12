/*
 ╔══════════════════════════════════════════════════════════════════╗
 ║  TX.ino  —  ESP32 Wroom  Bluetooth SPP + nRF24L01+PA+LNA       ║
 ║  Drone Ground Station Transmitter  v5.3                          ║
 ║  v5.3: removed portENTER_CRITICAL around radio.write() — it was  ║
 ║        unnecessary (single-threaded access) and was starving the ║
 ║        BT stack's interrupt servicing, causing SPP disconnects   ║
 ║        a few seconds after connecting.                           ║
 ║  Pairs with: Drone GS software (DC.py) via Bluetooth SPP        ║
 ║  Connects to: STM32 Flight Controller via nRF24L01              ║
 ╚══════════════════════════════════════════════════════════════════╝

 Wiring (nRF24L01+PA+LNA):
   VCC  → 3.3V  (use 10µF cap across VCC/GND — critical for PA variant!)
   GND  → GND
   CE   → GPIO4
   CSN  → GPIO5
   SCK  → GPIO18
   MOSI → GPIO23
   MISO → GPIO19

 IMPORTANT — PA+LNA power:
   The PA+LNA variant draws up to 130mA peak. Power from a dedicated 3.3V
   regulator (AMS1117-3.3 or similar), NOT directly from the ESP32 3V3 pin.
   Add 10µF electrolytic + 100nF ceramic cap between VCC and GND on nRF module.

 Libraries required (install via Arduino Library Manager):
   - RF24          by TMRh20  (v1.4.x or newer)
   - BluetoothSerial (built-in ESP32 Arduino core v2.x)

 Board: ESP32 Dev Module
 Partition: Default 4MB with spiffs  (or "No OTA (2MB APP/2MB SPIFFS)")
 CPU Frequency: 240 MHz

 ──────────────────────────────────────────────────────────────────
 NRF STATUS (reported back to software):
   NRF_STATE_INIT      = 0   hardware not yet started
   NRF_STATE_SELF_OK   = 1   radio.begin() passed, self-test OK
   NRF_STATE_CALLING   = 2   sending packets, awaiting RX ACK
   NRF_STATE_CONNECTED = 3   RX auto-ACK received (confirmed link)
 ──────────────────────────────────────────────────────────────────
*/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <BluetoothSerial.h>

// ─── nRF24 pins ──────────────────────────────────────────────────
#define NRF_CE   4
#define NRF_CSN  5

// ─── Force-Connect button ────────────────────────────────────────
#define FORCE_CONNECT_PIN 0   // GPIO0 / BOOT button (active LOW)
#define STATUS_LED        2   // Built-in LED (active LOW on most boards)

// ─── RF Channel & Address ────────────────────────────────────────
#define RF_CHANNEL   76
static const byte PIPE_ADDR[6] = "DRNGS";   // must match RX firmware

// ─── BT device name ──────────────────────────────────────────────
#define BT_DEVICE_NAME "DroneGS_TX"

// ─── Timing ──────────────────────────────────────────────────────
#define TX_INTERVAL_MS    20    // 50 Hz
#define ACK_REPORT_MS    200    // send ACK to software every 200ms
#define NRF_RETRY_MS    3000    // retry nRF init if not ready
#define BT_FAILSAFE_MS  2000    // zero pitch/roll/yaw if no BT data
#define FORCE_DEBOUNCE_MS 500

// ─── NRF state machine ───────────────────────────────────────────
enum NrfState {
  NRF_INIT      = 0,   // not started
  NRF_SELF_OK   = 1,   // hardware OK, not yet received ACK from RX
  NRF_CALLING   = 2,   // sending packets, awaiting ACK from RX
  NRF_CONNECTED = 3    // RX ACK confirmed
};

// ─── Packet structure (must match RX exactly) ────────────────────
struct __attribute__((packed)) TxPacket {
  uint16_t throttle;   // 1000–2000
  uint16_t yaw;        // 1000–2000
  uint16_t pitch;      // 1000–2000
  uint16_t roll;       // 1000–2000
  uint8_t  armed;      // 1 = armed
  uint8_t  flags;      // bit0=unused (was ESC_CAL, removed)  bit1=EMERGENCY_STOP  bit2=FORCE_NRF
  uint32_t seq;
  uint8_t  crc8;
};

// ─── Globals ─────────────────────────────────────────────────────
RF24            radio(NRF_CE, NRF_CSN);
BluetoothSerial SerialBT;

TxPacket pkt;
uint32_t seqCounter      = 0;
uint32_t lastTxMs        = 0;
uint32_t lastAckReportMs = 0;
uint32_t lastBTDataMs    = 0;
uint32_t lastLedMs       = 0;
uint32_t lastForceMs     = 0;
uint32_t lastNrfRetryMs  = 0;
uint32_t lastAckRxMs     = 0;   // when we last got an ACK from the RX

bool     btConnected     = false;
bool     forceConnect    = false;
NrfState nrfState        = NRF_INIT;

String   btBuf           = "";

// ─── CRC8 helper ─────────────────────────────────────────────────
uint8_t crc8_calc(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) crc ^= data[i];
  return crc;
}

// ─── LED helpers ─────────────────────────────────────────────────
void led_set(bool on) {
  digitalWrite(STATUS_LED, on ? LOW : HIGH);  // active-low
}

void led_flash(int times, int ms = 80) {
  for (int i = 0; i < times; i++) {
    led_set(true);  delay(ms);
    led_set(false); delay(ms);
  }
}

// ─── NRF state string helper ─────────────────────────────────────
const char* nrf_state_str(NrfState s) {
  switch (s) {
    case NRF_INIT:      return "INIT";
    case NRF_SELF_OK:   return "SELF_OK";
    case NRF_CALLING:   return "CALLING";
    case NRF_CONNECTED: return "CONNECTED";
    default:            return "UNKNOWN";
  }
}

// ─── nRF24 init ──────────────────────────────────────────────────
bool nrf_init() {
  // Make sure SPI pins are configured before radio.begin()
  SPI.begin(18, 19, 23, 5);   // SCK, MISO, MOSI, SS (SS unused but needed)
  delay(10);

  radio.powerDown();
  delay(5);

  if (!radio.begin()) {
    Serial.println("[NRF] radio.begin() FAILED — check wiring/power");
    nrfState = NRF_INIT;
    return false;
  }

  // Verify the radio is actually there (register sanity check)
  if (!radio.isChipConnected()) {
    Serial.println("[NRF] isChipConnected() FAILED");
    nrfState = NRF_INIT;
    return false;
  }

  radio.setPALevel(RF24_PA_LOW);      // Start LOW, raise after link confirmed
  radio.setDataRate(RF24_250KBPS);    // Max range
  radio.setChannel(RF_CHANNEL);
  radio.setRetries(5, 15);            // 5×250µs delay, 15 retries
  radio.openWritingPipe(PIPE_ADDR);
  radio.stopListening();
  radio.setAutoAck(true);
  radio.setCRCLength(RF24_CRC_16);
  radio.powerUp();
  delay(5);

  nrfState = NRF_SELF_OK;
  Serial.printf("[NRF] Initialized OK  ch=%d  state=%s\n",
                RF_CHANNEL, nrf_state_str(nrfState));

  if (btConnected) {
    SerialBT.printf("NRF_STATE:%d\n", (int)nrfState);
    SerialBT.printf("STATUS:NRF hardware OK — channel %d\n", RF_CHANNEL);
  }
  return true;
}

// ─── Force NRF reconnect ─────────────────────────────────────────
void handle_force_connect() {
  Serial.println("[NRF] Force reconnect initiated...");
  if (btConnected) SerialBT.println("STATUS:Force NRF reconnect...");

  NrfState prevState = nrfState;
  nrfState = NRF_INIT;
  if (btConnected) SerialBT.printf("NRF_STATE:%d\n", (int)nrfState);

  radio.powerDown();
  delay(150);

  if (nrf_init()) {
    radio.setPALevel(RF24_PA_MAX);   // Full power after forced reconnect
    Serial.println("[NRF] Force reconnect: OK");
    if (btConnected) SerialBT.println("STATUS:Force NRF OK — PA_MAX");
    led_flash(5);
  } else {
    Serial.println("[NRF] Force reconnect: FAILED");
    if (btConnected) SerialBT.println("STATUS:Force NRF FAILED — check module");
    led_flash(2, 300);
  }
  forceConnect = false;
  lastNrfRetryMs = millis();
}

// ─── Parse BT line from software ─────────────────────────────────
void parse_bt_line(const String &line) {
  if (line.length() == 0) return;

  if (line == "O") {
    pkt.armed ^= 1;
    Serial.printf("[BT] Armed=%d\n", pkt.armed);
    return;
  }
  if (line == "ESTOP") {
    pkt.throttle = 1000;
    pkt.armed    = 0;
    pkt.flags   |= 0x02;
    Serial.println("[BT] EMERGENCY STOP");
    return;
  }
  if (line == "FORCE_NRF") {
    forceConnect = true;
    Serial.println("[BT] Force NRF reconnect requested");
    return;
  }

  // Normal telemetry: T:xxxx,Y:xxxx,P:xxxx,R:xxxx
  if (line.startsWith("T:")) {
    int tIdx = line.indexOf("T:");
    int yIdx = line.indexOf(",Y:");
    int pIdx = line.indexOf(",P:");
    int rIdx = line.indexOf(",R:");
    if (tIdx < 0 || yIdx < 0 || pIdx < 0 || rIdx < 0) return;

    auto clamp16 = [](int v) -> uint16_t {
      return (uint16_t)constrain(v, 1000, 2000);
    };

    pkt.throttle = clamp16(line.substring(tIdx+2, yIdx).toInt());
    pkt.yaw      = clamp16(line.substring(yIdx+3, pIdx).toInt());
    pkt.pitch    = clamp16(line.substring(pIdx+3, rIdx).toInt());
    pkt.roll     = clamp16(line.substring(rIdx+3).toInt());
    lastBTDataMs = millis();
  }
}

// ─── nRF24 transmit (called at 50 Hz) ───────────────────────────
void nrf_transmit() {
  if (nrfState == NRF_INIT) return;

  // Advance state to CALLING once we start sending
  if (nrfState == NRF_SELF_OK) {
    nrfState = NRF_CALLING;
    if (btConnected) SerialBT.printf("NRF_STATE:%d\n", (int)nrfState);
    Serial.println("[NRF] State → CALLING");
  }

  pkt.seq  = seqCounter++;
  pkt.crc8 = crc8_calc((uint8_t*)&pkt, sizeof(pkt) - 1);

  // NOTE (v5.3): radio.write() is only ever called here, from loop() on the
  // main task — nothing else touches SPI concurrently (bt_callback() never
  // touches the radio), so no critical section was actually needed here.
  // Wrapping this blocking call (which can take several ms while it waits
  // through setRetries() retries) in portENTER_CRITICAL/portEXIT_CRITICAL
  // disabled interrupts on this core for that whole span, 50x/sec, which
  // can starve the Bluetooth Classic stack's interrupt servicing and
  // destabilize the SPP link (symptom: BT shows connected for a few
  // seconds then drops). Removed.
  bool ok = radio.write(&pkt, sizeof(pkt));

  if (ok) {
    lastAckRxMs = millis();
    if (nrfState != NRF_CONNECTED) {
      nrfState = NRF_CONNECTED;
      radio.setPALevel(RF24_PA_MAX);   // upgrade to full power on first ACK
      Serial.println("[NRF] State → CONNECTED (RX ACK received)");
      if (btConnected) {
        SerialBT.printf("NRF_STATE:%d\n", (int)nrfState);
        SerialBT.println("STATUS:NRF CONNECTED — RX responding");
      }
      led_flash(3, 60);
    }
  } else {
    // Lost ACK — downgrade state
    if (nrfState == NRF_CONNECTED) {
      uint32_t silentMs = millis() - lastAckRxMs;
      if (silentMs > 1000) {           // 1s without ACK → revert to CALLING
        nrfState = NRF_CALLING;
        if (btConnected) SerialBT.printf("NRF_STATE:%d\n", (int)nrfState);
        Serial.println("[NRF] State → CALLING (ACK lost)");
      }
    }
  }

  // Clear one-shot flags
  pkt.flags = 0;
}

// ─── BT callback ─────────────────────────────────────────────────
void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected  = true;
    lastBTDataMs = millis();
    Serial.println("[BT] Client connected");
    SerialBT.println("STATUS:CONNECTED");
    SerialBT.printf("NRF_STATE:%d\n", (int)nrfState);
  } else if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    Serial.println("[BT] Client disconnected — safety disarm");
    pkt.armed = 0;
  }
}

// ─── LED status (non-blocking, call from loop) ───────────────────
void led_task() {
  uint32_t now = millis();
  // Blink pattern encodes BT + NRF state:
  //   BT disconnected:                slow 1Hz blink
  //   BT connected, NRF INIT:         fast 5Hz blink
  //   BT connected, NRF SELF_OK:      2Hz blink
  //   BT connected, NRF CALLING:      double-blink
  //   BT connected, NRF CONNECTED:    solid on
  static bool ledOn = false;
  static uint8_t phase = 0;
  static uint32_t nextMs = 0;

  if (now < nextMs) return;

  if (!btConnected) {
    ledOn   = !ledOn;
    nextMs  = now + 500;
  } else if (nrfState == NRF_INIT) {
    ledOn  = !ledOn;
    nextMs = now + 100;
  } else if (nrfState == NRF_SELF_OK) {
    ledOn  = !ledOn;
    nextMs = now + 250;
  } else if (nrfState == NRF_CALLING) {
    // Double-blink pattern
    switch (phase % 4) {
      case 0: ledOn = true;  nextMs = now + 100; break;
      case 1: ledOn = false; nextMs = now + 100; break;
      case 2: ledOn = true;  nextMs = now + 100; break;
      case 3: ledOn = false; nextMs = now + 400; break;
    }
    phase++;
  } else {  // NRF_CONNECTED
    ledOn  = true;
    nextMs = now + 200;
  }

  led_set(ledOn);
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] Drone TX v5.3");

  pinMode(STATUS_LED,        OUTPUT);
  pinMode(FORCE_CONNECT_PIN, INPUT_PULLUP);
  led_set(false);

  // Default safe packet state
  pkt.throttle = 1000;
  pkt.yaw      = 1500;
  pkt.pitch    = 1500;
  pkt.roll     = 1500;
  pkt.armed    = 0;
  pkt.flags    = 0;
  pkt.seq      = 0;
  pkt.crc8     = 0;

  // ── Bluetooth SPP ────────────────────────────────────────────
  SerialBT.register_callback(bt_callback);
  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("[BT] FAILED to start Bluetooth!");
    // Flash SOS
    for (int i = 0; i < 9; i++) {
      led_flash(1, (i < 3 || i >= 6) ? 100 : 300);
      delay(100);
    }
  } else {
    Serial.printf("[BT] Started — device: %s\n", BT_DEVICE_NAME);
  }

  // ── nRF24 ────────────────────────────────────────────────────
  delay(100);  // let power settle
  if (!nrf_init()) {
    Serial.println("[NRF] Init failed — will retry in loop");
  }

  Serial.println("[BOOT] Setup complete");
  led_flash(3, 100);
}

// ═══════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  // ── Read BT serial ───────────────────────────────────────────
  while (SerialBT.available()) {
    char ch = (char)SerialBT.read();
    if (ch == '\n' || ch == '\r') {
      btBuf.trim();
      if (btBuf.length() > 0) {
        parse_bt_line(btBuf);
      }
      btBuf = "";
    } else {
      btBuf += ch;
      if (btBuf.length() > 128) btBuf = "";  // overflow guard
    }
  }

  // ── Force-connect button (hardware) ──────────────────────────
  if (digitalRead(FORCE_CONNECT_PIN) == LOW) {
    if (now - lastForceMs > FORCE_DEBOUNCE_MS) {
      lastForceMs  = now;
      forceConnect = true;
    }
  }
  if (forceConnect) {
    handle_force_connect();
  }

  // ── Auto-retry NRF init if not ready ─────────────────────────
  if (nrfState == NRF_INIT && (now - lastNrfRetryMs > NRF_RETRY_MS)) {
    lastNrfRetryMs = now;
    Serial.println("[NRF] Auto-retry init...");
    nrf_init();
  }

  // ── BT failsafe ──────────────────────────────────────────────
  if (btConnected && (now - lastBTDataMs) > BT_FAILSAFE_MS) {
    pkt.pitch = 1500;
    pkt.roll  = 1500;
    pkt.yaw   = 1500;
    // Throttle intentionally NOT reset (pilot may be hovering)
  }

  // ── 50 Hz TX ─────────────────────────────────────────────────
  if (now - lastTxMs >= TX_INTERVAL_MS) {
    lastTxMs = now;
    nrf_transmit();
  }

  // ── ACK report to software (200 ms) ──────────────────────────
  if (btConnected && (now - lastAckReportMs >= ACK_REPORT_MS)) {
    lastAckReportMs = now;
    bool nrfConn = (nrfState == NRF_CONNECTED);
    SerialBT.printf("ACK:T=%d,Y=%d,P=%d,R=%d,A=%d,NRF=%d,NRFS=%d\n",
      pkt.throttle, pkt.yaw, pkt.pitch, pkt.roll,
      pkt.armed,
      nrfConn ? 1 : 0,
      (int)nrfState);
  }

  // ── LED ──────────────────────────────────────────────────────
  led_task();

  // Small yield to avoid WDT
  delay(1);
}
