/*
 * StickS3 RF Tool  (M5StickS3 + Pingequa RF Pack S3)
 * Three modes, cycled with a long press of the front button:
 *   1. TESLA   - short press to transmit the charge-port-open signal at 433.92 MHz
 *   2. SUBGHZ  - live RSSI spectrum bar graph from the CC1101, around 433 MHz
 *   3. NRF     - live 2.4 GHz channel-activity bar graph from the nRF24L01
 *
 * No CC1101 or nRF24 library needed: both radios are driven with a small
 * built-in bit-banged SPI driver that shares the RF Pack S3's SPI bus, so the
 * only library required is M5Unified.
 *
 * Library (Library Manager): M5Unified
 * Board: M5StickS3 (or ESP32S3 Dev Module with USB CDC On Boot: Enabled)
 * Serial monitor: 115200 baud (prints CC1101 PARTNUM/VERSION at boot)
 *
 * Button has three tiers (all on the single front button):
 *   short press  (<500 ms)   - mode's primary action
 *   medium hold  (500-1500ms)- mode's secondary action
 *   long hold    (>1500 ms)  - switch to the next mode
 *
 * Layout follows Bruce's screen convention: a title bar at the top (screen
 * name, mode dots, battery %), the live content in the middle, and a hint
 * bar at the bottom (button legend / scale info).
 *
 * Changelog (12 features added on top of the base Tesla-opener/analyser):
 *   1. Battery percentage in the top bar
 *   2. Boot self-test: checks CC1101 and nRF24 before the main UI loads
 *   3. Three-tier button control (short / medium / long), replacing click/hold
 *   4. Beep feedback on every press, distinct tone on a successful Tesla send
 *   5. TESLA: runtime TX-power cycle (Low/Med/High) via medium hold
 *   6. TESLA: on-screen "opened N times this session" counter
 *   7. SUBGHZ: peak-hold overlay, decaying, resettable via medium hold
 *   8. SUBGHZ: marker line at 433.92 MHz (the Tesla frequency) for reference
 *   9. NRF: "busy channels" count shown in the hint bar each frame
 *  10. NRF: medium hold toggles fast-scan vs thorough-scan sample counts
 *  11. Auto screen-sleep after 30s idle; any press wakes it
 *  12. Low-battery warning: battery text turns red and blinks under 15%
 */

#include <M5Unified.h>
#include <LittleFS.h>

// ================== UI ==================
enum Mode { MODE_TESLA = 0, MODE_SUBGHZ, MODE_NRF, MODE_CAPTURE, MODE_COUNT };
static Mode mode = MODE_TESLA;

// ---------------- CONFIGURE THESE ----------------
// CC1101 / nRF24 pins: Bruce firmware's default for the M5StickS3 / RF Pack S3
#define PIN_SCK   5
#define PIN_MISO  4
#define PIN_MOSI  6
#define PIN_CSN   2     // CC1101 CS
#define PIN_GDO0  3     // CC1101 GDO0

#define PIN_NRF_CS   8    // nRF24 CS
#define PIN_NRF_CE   1    // nRF24 CE

#define FREQ_MHZ   433.92f   // Tesla signal frequency, AU/EU. (US/Canada cars use 315.00)
#define REPEATS    3         // repeats of each burst (each burst is ~134 ms)
#define GAP_US     30000     // silence between repeats

// Which captured burst to send: 1 = capture 1/3, 2 = capture 2, 3 = both (default)
#define VARIANT 3

// TX power levels cycled with a medium hold in TESLA mode (CC1101 PA table values)
static const uint8_t TX_POWER_LEVELS[] = { 0x60 /*low, ~0dBm*/, 0x8E /*med, ~5dBm*/, 0xC0 /*high, ~10dBm*/ };
static const char*   TX_POWER_NAMES[]  = { "Low", "Med", "High" };
static uint8_t txPowerIdx = 2;   // start on High

// Sub-GHz spectrum sweep (CC1101). Stays on the same ISM band as the Tesla signal;
// widen SUBGHZ_START/END if you want to look elsewhere (keep inside the chip's bands:
// ~300-348, 387-464 or 779-928 MHz for the common 26 MHz CC1101).
#define SUBGHZ_START_MHZ  420.0f
#define SUBGHZ_END_MHZ    448.0f
#define SUBGHZ_POINTS     48        // bars across the screen
#define SUBGHZ_SETTLE_US  3000      // time to let RSSI settle after retuning
#define PEAK_DECAY_PER_FRAME 2      // dB the peak-hold line falls back each frame

// 2.4 GHz spectrum sweep (nRF24). Channel N = 2400 + N MHz, 0-125.
#define NRF_CH_FIRST   0
#define NRF_CH_LAST    125
#define NRF_BARS       42           // bars across the screen (channels grouped into bins)
static uint8_t nrfSamples = 12;     // runtime-adjustable: fast scan vs thorough scan
#define NRF_SAMPLES_FAST      5
#define NRF_SAMPLES_THOROUGH  20

#define IDLE_SLEEP_MS  30000        // screen dims after this long with no button press

// Capture & replay (CC1101). Read RAW records every demodulated OOK edge at
// CAPTURE_FREQ_MHZ until Stop is pressed, then replays that exact timing stream.
#define CAPTURE_FREQ_MHZ     FREQ_MHZ   // change if you want to capture elsewhere
#define CAPTURE_MAX_PULSES   4096       // 8 KiB RAM; 16+ typical remote transmissions
#define REPLAY_IRQ_WINDOW_US 8000       // longest interval with interrupts masked
#define REPLAY_TASK_YIELD_US 1000000UL  // keep FreeRTOS alive during unusually long RAW TX
// -------------------------------------------------

// ---------- Captured signal (no need to edit) ----------
// Tesla charge-port-open signal, 433.92 MHz OOK.
// Timings in microseconds, alternating MARK (carrier on) / SPACE (off), starting with a MARK.
// From three Flipper Zero RAW captures (files 1 and 3 were identical).
// Each burst = preamble + 3 data frames, trimmed from the clean part of the capture.

// Capture 1/3 (Flipper preset OOK650)
const uint16_t TESLA_A[] = {
  347, 454, 351, 426, 383, 430, 351, 434, 351, 438, 381, 408,
  381, 440, 351, 442, 349, 442, 349, 442, 381, 410, 379, 412,
  379, 1202, 381, 414, 379, 414, 807, 784, 393, 416, 791, 786,
  791, 822, 361, 414, 795, 818, 759, 826, 793, 792, 795, 798,
  795, 794, 401, 400, 785, 400, 379, 828, 771, 404, 379, 830,
  377, 394, 791, 408, 393, 398, 393, 796, 405, 396, 397, 394,
  795, 406, 395, 792, 799, 404, 395, 792, 799, 800, 401, 382,
  403, 416, 357, 418, 785, 418, 383, 816, 385, 412, 775, 1182,
  409, 400, 393, 398, 817, 780, 385, 412, 779, 808, 811, 782,
  391, 418, 791, 784, 787, 824, 793, 794, 793, 794, 795, 794,
  401, 400, 787, 402, 395, 790, 801, 402, 397, 788, 403, 396,
  791, 404, 395, 394, 395, 792, 403, 380, 403, 410, 805, 378,
  411, 802, 775, 406, 379, 832, 771, 800, 407, 396, 395, 394,
  397, 396, 795, 406, 395, 792, 405, 396, 789, 1198, 395, 396,
  395, 398, 785, 816, 387, 410, 777, 810, 777, 812, 389, 416,
  789, 784, 789, 824, 795, 794, 795, 794, 795, 794, 401, 398,
  787, 402, 397, 788, 801, 402, 395, 788, 405, 396, 791, 404,
  395, 396, 393, 792, 405, 380, 401, 410, 803, 378, 411, 802,
  775, 410, 393, 792, 803, 802, 403, 366, 421, 378, 403, 410,
  803, 378, 409, 806, 377, 412, 409, 1176,
};
const size_t TESLA_A_LEN = 236;

// Capture 2 (Flipper preset OOK270)
const uint16_t TESLA_B[] = {
  391, 400, 389, 438, 385, 408, 387, 408, 387, 408, 387, 408,
  385, 408, 387, 408, 387, 408, 387, 410, 387, 408, 387, 410,
  387, 1170, 419, 410, 385, 376, 815, 778, 415, 384, 811, 778,
  811, 782, 425, 376, 797, 802, 799, 794, 797, 796, 797, 794,
  797, 796, 401, 380, 799, 392, 407, 794, 801, 402, 365, 820,
  403, 396, 789, 404, 395, 394, 393, 794, 407, 396, 397, 396,
  791, 406, 415, 768, 787, 396, 409, 796, 801, 800, 403, 368,
  423, 398, 393, 396, 791, 406, 395, 788, 407, 396, 791, 1192,
  409, 404, 379, 406, 803, 776, 413, 384, 805, 786, 789, 822,
  399, 382, 793, 786, 821, 792, 797, 794, 795, 796, 793, 796,
  397, 380, 795, 404, 411, 802, 773, 406, 379, 798, 411, 396,
  793, 408, 395, 394, 395, 794, 407, 394, 395, 396, 791, 404,
  379, 798, 809, 406, 395, 790, 803, 796, 401, 380, 399, 410,
  407, 380, 803, 410, 379, 802, 415, 358, 799, 1202, 391, 400,
  389, 436, 779, 806, 379, 418, 779, 810, 781, 784, 395, 410,
  795, 802, 801, 798, 797, 794, 795, 792, 795, 798, 401, 382,
  801, 388, 409, 792, 799, 402, 397, 788, 405, 396, 791, 404,
  395, 394, 395, 792, 403, 380, 401, 410, 805, 378, 411, 802,
  773, 408, 393, 794, 805, 802, 403, 380, 401, 420, 355, 418,
  783, 416, 385, 818, 385, 408, 395, 1186,
};
const size_t TESLA_B_LEN = 236;

// -------------------------------------------------

static void drawStatusFwd(const char* line1, const char* line2 = "");   // defined in the UI section below
static void enterMode(Mode m);                                           // defined after capture/replay

// ================== Shared bit-banged SPI ==================
static uint8_t spiByte(uint8_t v) {
  uint8_t r = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_MOSI, (v >> i) & 1);
    delayMicroseconds(1);
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(1);
    r = (r << 1) | (digitalRead(PIN_MISO) ? 1 : 0);
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(1);
  }
  return r;
}

static void busInit() {
  pinMode(PIN_SCK, OUTPUT);  digitalWrite(PIN_SCK, LOW);
  pinMode(PIN_MOSI, OUTPUT); digitalWrite(PIN_MOSI, LOW);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_CSN, OUTPUT);    digitalWrite(PIN_CSN, HIGH);
  pinMode(PIN_NRF_CS, OUTPUT); digitalWrite(PIN_NRF_CS, HIGH);
  pinMode(PIN_NRF_CE, OUTPUT); digitalWrite(PIN_NRF_CE, LOW);
  pinMode(PIN_GDO0, INPUT);                 // only driven by us while the CC1101 transmits
}

// ================== CC1101 driver ==================
static bool waitMisoLow() {                       // CC1101 pulls MISO low when ready
  uint32_t t = micros();
  while (digitalRead(PIN_MISO)) {
    if (micros() - t > 5000) return false;
  }
  return true;
}

static void ccBegin() { digitalWrite(PIN_CSN, LOW); waitMisoLow(); }
static void ccEnd()   { digitalWrite(PIN_CSN, HIGH); }

static uint8_t ccStrobe(uint8_t cmd) {
  ccBegin(); uint8_t s = spiByte(cmd); ccEnd(); return s;
}
static void ccWrite(uint8_t reg, uint8_t val) {
  ccBegin(); spiByte(reg); spiByte(val); ccEnd();
}
static uint8_t ccReadStatus(uint8_t reg) {        // status registers need the burst bit
  ccBegin(); spiByte(reg | 0xC0); uint8_t v = spiByte(0); ccEnd(); return v;
}
static void ccWritePATable(uint8_t level) {       // OOK: index 0 = off, index 1 = on
  ccBegin(); spiByte(0x7E);
  for (int i = 0; i < 8; i++) spiByte(i == 1 ? level : 0x00);
  ccEnd();
}
static void ccReset() {
  digitalWrite(PIN_CSN, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_CSN, LOW);  delayMicroseconds(10);
  digitalWrite(PIN_CSN, HIGH); delayMicroseconds(50);
  ccBegin(); spiByte(0x30); /* SRES */ waitMisoLow(); ccEnd();
  delay(5);
}

static void ccSetFreqMHz(float mhz) {
  uint32_t f = (uint32_t)((double)mhz * 65536.0 / 26.0 + 0.5);   // 26 MHz crystal
  ccWrite(0x0D, (uint8_t)(f >> 16));   // FREQ2
  ccWrite(0x0E, (uint8_t)(f >> 8));    // FREQ1
  ccWrite(0x0F, (uint8_t)(f));         // FREQ0
}

static bool ccPresent = false, nrfPresent = false;
static bool radioOk = false;
static uint8_t lastPart = 0, lastVersion = 0;

static bool radioInit() {
  busInit();
  delay(100);
  ccReset();
  lastPart    = ccReadStatus(0x30);         // PARTNUM (0x00 for CC1101)
  lastVersion = ccReadStatus(0x31);         // VERSION (0x14 normally, 0x04 on old silicon)
  Serial.printf("CC1101 PARTNUM=0x%02X VERSION=0x%02X (SCK=%d MISO=%d MOSI=%d CS=%d GDO0=%d)\n",
                lastPart, lastVersion, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN, PIN_GDO0);
  if (lastVersion == 0x00 || lastVersion == 0xFF) return false;

  // Register set: 433 MHz OOK, async serial TX (same idea as Flipper's OOK650 preset)
  const uint8_t cfg[][2] = {
    {0x02, 0x0D},               // IOCFG0: async serial data on GDO0
    {0x03, 0x47},               // FIFOTHR
    {0x08, 0x32},               // PKTCTRL0: async serial mode, infinite length
    {0x0B, 0x06},               // FSCTRL1
    {0x0C, 0x00},               // FSCTRL0
    {0x10, 0x67},               // MDMCFG4
    {0x11, 0x32},               // MDMCFG3
    {0x12, 0x30},               // MDMCFG2: ASK/OOK, no sync
    {0x13, 0x00},               // MDMCFG1
    {0x14, 0x00},               // MDMCFG0
    {0x15, 0x15},               // DEVIATN
    {0x17, 0x00},               // MCSM1: go to IDLE after TX/RX
    {0x18, 0x18},               // MCSM0: autocalibrate idle -> TX/RX
    {0x19, 0x18},               // FOCCFG
    {0x21, 0xB6},               // FREND1
    {0x22, 0x11},               // FREND0: use PA table entry 1 for "on"
    {0x23, 0xE9}, {0x24, 0x2A}, {0x25, 0x00}, {0x26, 0x1F},   // FSCAL3..0
  };
  for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) ccWrite(cfg[i][0], cfg[i][1]);
  ccSetFreqMHz(FREQ_MHZ);
  ccWritePATable(TX_POWER_LEVELS[txPowerIdx]);
  ccStrobe(0x36);                           // SIDLE
  return true;
}

static bool ccTxStart(float freqMHz) {
  ccSetFreqMHz(freqMHz);
  ccStrobe(0x36);                           // SIDLE
  ccStrobe(0x35);                           // STX (calibrates, then transmits)
  uint32_t t = millis();
  while ((ccReadStatus(0x35) & 0x1F) != 0x13) {   // MARCSTATE 0x13 = TX
    if (millis() - t > 20) return false;
    delayMicroseconds(200);
  }
  pinMode(PIN_GDO0, OUTPUT);
  digitalWrite(PIN_GDO0, LOW);
  return true;
}

static void ccTxStop() {
  digitalWrite(PIN_GDO0, LOW);
  pinMode(PIN_GDO0, INPUT);                 // hand the pin back to the CC1101
  ccStrobe(0x36);                           // SIDLE
}

// Keep the waveform timing deterministic without masking interrupts for a whole
// captured signal.  A stored capture can be much longer than the built-in Tesla
// burst; masking interrupts for all of it can trip the ESP32 interrupt watchdog
// during replay and reset the StickS3.
static void sendBurst(const uint16_t* t, size_t n, bool startsMark = true) {
  noInterrupts();
  uint32_t start = micros(), acc = 0, irqWindowStart = start, taskYieldStart = start;
  for (size_t i = 0; i < n; i++) {
    bool mark = ((i % 2) == 0) ? startsMark : !startsMark;
    digitalWrite(PIN_GDO0, mark ? HIGH : LOW);
    acc += t[i];
    while ((uint32_t)(micros() - start) < acc) {
      // Let the radio/FreeRTOS interrupts run briefly. The output pin keeps its
      // current level, and shifting start preserves the timing of later pulses.
      uint32_t now = micros();
      if ((uint32_t)(now - irqWindowStart) >= REPLAY_IRQ_WINDOW_US) {
        interrupts();
        if ((uint32_t)(now - taskYieldStart) >= REPLAY_TASK_YIELD_US) {
          yield();
          taskYieldStart = micros();
        }
        delayMicroseconds(2);
        uint32_t resumed = micros();
        start += resumed - now;
        irqWindowStart = resumed;
        noInterrupts();
      }
    }
  }
  digitalWrite(PIN_GDO0, LOW);
  interrupts();
}

static uint32_t sendCount = 0;   // feature 6: "opened N times this session"

static void beep(uint16_t freqHz, uint16_t ms) {
  M5.Speaker.tone(freqHz, ms);   // feature 4: button/feedback beeps
}

static void openChargePort() {
  if (!radioOk) { return; }   // caller already checked / shown the error
  if (!ccTxStart(FREQ_MHZ)) { drawStatusFwd("TX failed", "CC1101 did not enter TX"); ccTxStop(); beep(220, 200); return; }
  drawStatusFwd("Sending...");
  for (int r = 0; r < REPEATS; r++) {
#if VARIANT == 1 || VARIANT == 3
    sendBurst(TESLA_A, TESLA_A_LEN);
    delayMicroseconds(GAP_US);
#endif
#if VARIANT == 2 || VARIANT == 3
    sendBurst(TESLA_B, TESLA_B_LEN);
    delayMicroseconds(GAP_US);
#endif
  }
  ccTxStop();
  sendCount++;
  beep(1800, 120);                          // feature 4: distinct "sent" tone
  drawStatusFwd("Sent", "Car must be unlocked");
}

// Sub-GHz RSSI sweep. Returns a signed dBm estimate per bar into out[].
static void ccSpectrumSweep(int8_t* out, int points) {
  ccStrobe(0x36);                           // SIDLE
  for (int i = 0; i < points; i++) {
    float mhz = SUBGHZ_START_MHZ + (SUBGHZ_END_MHZ - SUBGHZ_START_MHZ) * i / (points - 1);
    ccSetFreqMHz(mhz);
    ccStrobe(0x34);                         // SRX
    delayMicroseconds(SUBGHZ_SETTLE_US);

    int total = 0;
    for (int s = 0; s < 5; s++) {
      total += ccReadStatus(0x34);
      delayMicroseconds(200);
    }
    uint8_t raw = total / 5;                // averaged RSSI

    ccStrobe(0x36);                         // SIDLE before retuning
    int rssi = raw;
    int dbm = (rssi >= 128) ? ((rssi - 256) / 2 - 74) : (rssi / 2 - 74);
    if (dbm < -100) dbm = -100;
    if (dbm > 20)   dbm = 20;
    out[i] = (int8_t)dbm;
  }
  ccSetFreqMHz(FREQ_MHZ);                   // park back on the Tesla frequency when idle
}

// ================== nRF24 driver ==================
static void nrfBegin() { digitalWrite(PIN_NRF_CS, LOW); }
static void nrfEnd()   { digitalWrite(PIN_NRF_CS, HIGH); }

static void nrfWriteReg(uint8_t reg, uint8_t val) {
  nrfBegin(); spiByte(0x20 | (reg & 0x1F)); spiByte(val); nrfEnd();
}
static uint8_t nrfReadReg(uint8_t reg) {
  nrfBegin(); spiByte(reg & 0x1F); uint8_t v = spiByte(0xFF); nrfEnd(); return v;
}

static bool nrfDetect() {
  // No STATUS/VERSION register on nRF24, so detect by writing then reading back
  // a scratch value in RF_CH (a plain read/write register, 0-127 valid).
  nrfWriteReg(0x05, 0x17);
  uint8_t v = nrfReadReg(0x05);
  return v == 0x17;
}

static void nrfInit() {
  digitalWrite(PIN_NRF_CE, LOW);
  delay(5);
  nrfWriteReg(0x00, 0x0F);   // CONFIG: PWR_UP, PRIM_RX, mask all IRQs
  nrfWriteReg(0x01, 0x00);   // EN_AA: no auto-ack (not needed for a scanner)
  nrfWriteReg(0x02, 0x00);   // EN_RXADDR: no pipes enabled, we just want RPD per channel
  nrfWriteReg(0x03, 0x03);   // SETUP_AW: 5-byte addresses
  nrfWriteReg(0x04, 0x00);   // SETUP_RETR: none
  nrfWriteReg(0x06, 0x0F);   // RF_SETUP: 1 Mbps, max power
  delay(2);
}

// RPD ("received power detector", legacy name CD) = register 0x09 bit 0.
// True if a signal above ~-64 dBm was seen on the current channel recently.
static bool nrfCarrierOnChannel(uint8_t channel) {
  digitalWrite(PIN_NRF_CE, LOW);
  nrfWriteReg(0x05, channel);         // RF_CH
  digitalWrite(PIN_NRF_CE, HIGH);
  delayMicroseconds(170);             // datasheet: >= 170us to settle in RX
  bool hit = nrfReadReg(0x09) & 0x01; // RPD
  digitalWrite(PIN_NRF_CE, LOW);
  return hit;
}

static void nrfSpectrumSweep(uint8_t* out, int bars) {
  const int span = NRF_CH_LAST - NRF_CH_FIRST + 1;
  for (int b = 0; b < bars; b++) {
    int chLo = NRF_CH_FIRST + (span * b) / bars;
    int chHi = NRF_CH_FIRST + (span * (b + 1)) / bars;
    if (chHi <= chLo) chHi = chLo + 1;
    uint8_t hits = 0;
    for (int s = 0; s < nrfSamples; s++) {
      int ch = chLo + ((chHi - chLo) ? (s % (chHi - chLo)) : 0);
      if (nrfCarrierOnChannel((uint8_t)ch)) hits++;
    }
    out[b] = hits;
  }
}

// ================== Bruce-style frame (title bar / content / hint bar) ==================
static const uint16_t BAR_BG   = 0x18E3;   // dark slate
static const uint16_t GRID_COL = 0x4A69;   // dim gridline
static const int TOPBAR_H = 16;
static const int HINTBAR_H = 12;
static int contentTop, contentBottom, contentLeft, contentRight;

static const char* MODE_NAMES[MODE_COUNT] = { "TESLA", "SUBGHZ", "NRF24", "CAPTURE" };

static void drawModeDots(int rightEdge) {
  auto& d = M5.Display;
  int dotR = 2, gap = 7;
  int x = rightEdge - (MODE_COUNT - 1) * gap;
  int y = TOPBAR_H / 2;
  for (int i = 0; i < MODE_COUNT; i++) {
    uint16_t c = (i == (int)mode) ? TFT_WHITE : GRID_COL;
    d.fillCircle(x + i * gap, y, dotR, c);
  }
}

// feature 1 + 12: battery percentage, red/blinking under 15%
static void drawBattery(int rightEdge) {
  auto& d = M5.Display;
  int32_t pct = M5.Power.getBatteryLevel();   // 0-100, or -1 if unknown
  char buf[8];
  if (pct < 0) { snprintf(buf, sizeof(buf), "USB"); }
  else         { snprintf(buf, sizeof(buf), "%ld%%", (long)pct); }
  bool low = (pct >= 0 && pct < 15);
  bool blinkOn = ((millis() / 500) % 2) == 0;
  uint16_t col = (low && !blinkOn) ? BAR_BG : (low ? TFT_RED : TFT_SILVER);
  d.setTextColor(col, BAR_BG);
  d.setTextDatum(middle_right);
  d.setTextSize(1);
  d.drawString(buf, rightEdge, TOPBAR_H / 2);
}

// Clears the screen and draws the Bruce-style title bar; sets the content rect.
static void beginFrame(const char* title) {
  auto& d = M5.Display;
  int w = d.width(), h = d.height();
  d.fillScreen(TFT_BLACK);
  d.fillRect(0, 0, w, TOPBAR_H, BAR_BG);
  d.drawFastHLine(0, TOPBAR_H, w, GRID_COL);
  d.setTextColor(TFT_WHITE, BAR_BG);
  d.setTextDatum(middle_left);
  d.setTextSize(1);
  d.drawString(title, 4, TOPBAR_H / 2);
  int battRight = w - 4;
  drawBattery(battRight);
  drawModeDots(battRight - 28);              // dots sit left of the battery text
  contentLeft = 0; contentRight = w;
  contentTop = TOPBAR_H + 1;
  contentBottom = h - HINTBAR_H - 1;
}

// Draws the Bruce-style hint/legend bar at the bottom of the screen.
static void drawHint(const char* left, const char* right) {
  auto& d = M5.Display;
  int w = d.width(), h = d.height();
  int y0 = h - HINTBAR_H;
  d.fillRect(0, y0, w, HINTBAR_H, BAR_BG);
  d.drawFastHLine(0, y0, w, GRID_COL);
  d.setTextColor(TFT_SILVER, BAR_BG);
  d.setTextSize(1);
  if (left && left[0]) { d.setTextDatum(middle_left);  d.drawString(left,  3, y0 + HINTBAR_H / 2); }
  if (right && right[0]) { d.setTextDatum(middle_right); d.drawString(right, w - 3, y0 + HINTBAR_H / 2); }
}

static void drawStatus(const char* line1, const char* line2 = "") {
  beginFrame(MODE_NAMES[mode]);
  auto& d = M5.Display;
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextDatum(middle_center);
  int midX = (contentLeft + contentRight) / 2;
  int midY = (contentTop + contentBottom) / 2;
  d.setTextSize(2);
  d.drawString(line1, midX, midY - 10);
  d.setTextSize(1);
  d.drawString(line2, midX, midY + 12);
  char left[24];
  snprintf(left, sizeof(left), "Opened: %lu", (unsigned long)sendCount);   // feature 6
  drawHint(left, "Short:Send Med:Pwr");
}
// Forward-callable alias so the CC1101 TX routines (defined above the UI section) can draw status.
static void drawStatusFwd(const char* line1, const char* line2) { drawStatus(line1, line2); }

static void showNotFound() {
  char buf[40];
  snprintf(buf, sizeof(buf), "PART=0x%02X VER=0x%02X (0x14=OK)", lastPart, lastVersion);
  beginFrame(MODE_NAMES[mode]);
  auto& d = M5.Display;
  d.setTextColor(TFT_RED, TFT_BLACK);
  d.setTextDatum(middle_center);
  int midX = (contentLeft + contentRight) / 2, midY = (contentTop + contentBottom) / 2;
  d.setTextSize(2);
  d.drawString("CC1101 not found", midX, midY - 10);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextSize(1);
  d.drawString(buf, midX, midY + 12);
  drawHint("Short: Retry", "Long: Next");
}

static void showReady() {
  char buf[40], pwr[24];
  snprintf(buf, sizeof(buf), "CC1101 PART=0x%02X VER=0x%02X", lastPart, lastVersion);
  snprintf(pwr, sizeof(pwr), "TX:%s", TX_POWER_NAMES[txPowerIdx]);
  beginFrame(MODE_NAMES[mode]);
  auto& d = M5.Display;
  d.setTextColor(TFT_GREEN, TFT_BLACK);
  d.setTextDatum(middle_center);
  int midX = (contentLeft + contentRight) / 2, midY = (contentTop + contentBottom) / 2;
  d.setTextSize(2);
  d.drawString("Charge port", midX, midY - 10);
  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.setTextSize(1);
  d.drawString(buf, midX, midY + 12);
  d.setTextDatum(top_right);
  d.drawString(pwr, contentRight - 2, contentTop + 2);
  char left[24];
  snprintf(left, sizeof(left), "Opened: %lu", (unsigned long)sendCount);
  drawHint(left, "Short:Send Med:Pwr");
}

// feature 5: cycles TX power and re-writes the CC1101 PA table live
static void cycleTxPower() {
  txPowerIdx = (txPowerIdx + 1) % (sizeof(TX_POWER_LEVELS) / sizeof(TX_POWER_LEVELS[0]));
  if (radioOk) ccWritePATable(TX_POWER_LEVELS[txPowerIdx]);
  beep(1200, 60);
  showReady();
}

// Generic scaled bar-graph spectrum display: y-axis gridlines/labels for minVal..maxVal,
// bars for heights[0..n), optional peak-hold overlay, and a bottom hint bar with the
// x-axis frequency range and the live peak.
static void drawSpectrum(const char* title, const int* heights, const int8_t* peakHold, int n,
                          int minVal, int maxVal, const char* unit,
                          const char* xLeftLabel, const char* rightHint,
                          float xLeftVal, float xRightVal, bool xIsFreqMHz,
                          float markerFreqMHz) {
  beginFrame(title);
  auto& d = M5.Display;

  const int yAxisW = 26;               // left margin reserved for value labels
  int plotLeft  = contentLeft + yAxisW;
  int plotRight = contentRight - 1;
  int plotTop   = contentTop + 1;
  int plotBottom = contentBottom - 1;
  int plotH = plotBottom - plotTop;
  int plotW = plotRight - plotLeft;
  if (plotH < 1 || plotW < 1 || n <= 0) return;

  // --- y-axis gridlines + labels (4 lines: max, 2/3, 1/3, min) ---
  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.setTextSize(1);
  d.setTextDatum(middle_right);
  const int GRIDLINES = 4;
  for (int g = 0; g <= GRIDLINES; g++) {
    int y = plotTop + (plotH * g) / GRIDLINES;
    int val = maxVal - ((maxVal - minVal) * g) / GRIDLINES;
    d.drawFastHLine(plotLeft, y, plotW, (g == GRIDLINES) ? GRID_COL : 0x2104);
    char lbl[8];
    snprintf(lbl, sizeof(lbl), "%d", val);
    d.drawString(lbl, plotLeft - 2, (g == GRIDLINES) ? y - 5 : y);
  }
  d.drawFastVLine(plotLeft, plotTop, plotH, GRID_COL);

  int range = maxVal - minVal; if (range <= 0) range = 1;
  int barW = plotW / n; if (barW < 1) barW = 1;

  // --- feature 8: marker line at the Tesla frequency, SUBGHZ only ---
  if (xIsFreqMHz && markerFreqMHz > 0 && markerFreqMHz >= xLeftVal && markerFreqMHz <= xRightVal) {
    int mx = plotLeft + (int)((markerFreqMHz - xLeftVal) / (xRightVal - xLeftVal) * plotW);
    d.drawFastVLine(mx, plotTop, plotH, TFT_CYAN);
    d.setTextColor(TFT_CYAN, TFT_BLACK);
    d.setTextDatum(top_left);
    d.drawString("Tesla", mx + 2, plotTop + 1);
  }

  // --- bars ---
  int peakIdx = 0, peakVal = heights[0];
  for (int i = 0; i < n; i++) {
    int v = heights[i];
    if (v > peakVal) { peakVal = v; peakIdx = i; }
    if (v < minVal) v = minVal;
    if (v > maxVal) v = maxVal;
    int barH = (plotH * (v - minVal)) / range;
    int x = plotLeft + i * barW;
    uint16_t color = ((v - minVal) * 2 >= range) ? TFT_RED : (v > minVal ? TFT_YELLOW : TFT_DARKGREEN);
    d.fillRect(x, plotBottom - barH, max(1, barW - 1), barH, color);
  }

  // --- feature 7: peak-hold overlay (thin white line), SUBGHZ only ---
  if (peakHold) {
    for (int i = 0; i < n; i++) {
      int v = peakHold[i];
      if (v < minVal) v = minVal;
      if (v > maxVal) v = maxVal;
      int ph = plotBottom - (plotH * (v - minVal)) / range;
      int x = plotLeft + i * barW;
      d.drawFastHLine(x, ph, max(1, barW - 1), TFT_WHITE);
    }
  }

  // --- x-axis / peak readout in the hint bar ---
  char left[24], right[28];
  snprintf(left, sizeof(left), "%s", xLeftLabel);
  if (xIsFreqMHz) {
    float peakFreq = xLeftVal + (xRightVal - xLeftVal) * peakIdx / (float)(n - 1);
    snprintf(right, sizeof(right), "Pk %.1fMHz %d%s", peakFreq, peakVal, unit);
  } else if (rightHint && rightHint[0]) {
    snprintf(right, sizeof(right), "%s Pk ch%d %d%s", rightHint, peakIdx, peakVal, unit);
  } else {
    snprintf(right, sizeof(right), "Pk ch%d %d%s", peakIdx, peakVal, unit);
  }
  drawHint(left, right);
}

static int8_t subghzPeak[SUBGHZ_POINTS];
static bool   subghzPeakInit = false;

static void runSubghzFrame() {
  static int8_t levels[SUBGHZ_POINTS];
  ccSpectrumSweep(levels, SUBGHZ_POINTS);

  if (!subghzPeakInit) { for (int i = 0; i < SUBGHZ_POINTS; i++) subghzPeak[i] = levels[i]; subghzPeakInit = true; }
  for (int i = 0; i < SUBGHZ_POINTS; i++) {                      // feature 7: decaying peak-hold
    if (levels[i] > subghzPeak[i]) subghzPeak[i] = levels[i];
    else if (subghzPeak[i] > levels[i]) subghzPeak[i] -= PEAK_DECAY_PER_FRAME;
    if (subghzPeak[i] < levels[i]) subghzPeak[i] = levels[i];
  }

  int heights[SUBGHZ_POINTS];
  for (int i = 0; i < SUBGHZ_POINTS; i++) heights[i] = levels[i];
  char lo[16];
  snprintf(lo, sizeof(lo), "%.0fMHz", SUBGHZ_START_MHZ);
  drawSpectrum("SUBGHZ", heights, subghzPeak, SUBGHZ_POINTS, -100, 20, "dBm",
               lo, "", SUBGHZ_START_MHZ, SUBGHZ_END_MHZ, true, FREQ_MHZ);
}

static void runNrfFrame() {
  static uint8_t hits[NRF_BARS];
  nrfSpectrumSweep(hits, NRF_BARS);
  int heights[NRF_BARS];
  int busy = 0;                                                  // feature 9: busy-channel count
  for (int i = 0; i < NRF_BARS; i++) { heights[i] = hits[i]; if (hits[i] > 0) busy++; }
  char busyStr[16];
  snprintf(busyStr, sizeof(busyStr), "Busy:%d", busy);
  drawSpectrum("NRF24", heights, nullptr, NRF_BARS, 0, nrfSamples, "hit",
               "2400MHz", busyStr, NRF_CH_FIRST, NRF_CH_LAST, false, -1);
}

// feature 10: fast vs thorough nRF scan
static void toggleNrfScanSpeed() {
  nrfSamples = (nrfSamples == NRF_SAMPLES_FAST) ? NRF_SAMPLES_THOROUGH : NRF_SAMPLES_FAST;
  beep(1200, 60);
  beginFrame("NRF24");
  char buf[20]; snprintf(buf, sizeof(buf), "%s scan",
                         nrfSamples == NRF_SAMPLES_FAST ? "Fast" : "Thorough");
  drawHint(buf, "");
}

static uint16_t captureBuf[CAPTURE_MAX_PULSES];
static size_t   captureLen = 0;
static bool     captureHasData = false;
static bool     captureLastFailed = false;
static float    captureFreqMHz = CAPTURE_FREQ_MHZ;   // actual frequency of the stored capture
static bool     captureStartsMark = true;

// Read RAW is edge driven, not a blocking "next burst" operation. Capturing in
// the GPIO ISR preserves the sub-millisecond timings that polling from loop()
// would lose, while the UI remains available to stop recording manually.
static volatile bool     captureRecording = false;
static volatile bool     captureInterruptAttached = false;
static volatile bool     captureStarted = false;
static volatile bool     captureBufferFull = false;
static volatile size_t   captureIsrLen = 0;
static volatile bool     captureIsrStartsMark = true;
static volatile uint32_t captureEdgeUs = 0;
static bool              sideReplayActive = false;
static bool              sideReplayRequested = false;

// Single-file .sub save/replace in the StickS3's own internal flash (LittleFS) -
// no SD card needed. Written in the same RAW_Data text format as a real Flipper
// Zero .sub file, so the saved file is compatible if you ever pull it off and
// open it elsewhere. Always the same filename, so saving again overwrites it.
static const char* CAPTURE_FILE = "/capture.sub";

static void IRAM_ATTR onCaptureEdge() {
  if (!captureRecording) return;

  const uint32_t now = micros();
  const bool levelIsMark = digitalRead(PIN_GDO0) == HIGH;

  // The first edge establishes a real signal boundary. It avoids recording the
  // arbitrary quiet time between pressing REC and pressing the remote.
  if (!captureStarted) {
    captureStarted = true;
    captureIsrStartsMark = levelIsMark;
    captureEdgeUs = now;
    return;
  }

  const uint32_t duration = now - captureEdgeUs;
  const size_t index = captureIsrLen;
  if (index >= CAPTURE_MAX_PULSES) {
    captureBufferFull = true;
    captureRecording = false;
    return;
  }

  // The flash format and replay buffer use 16-bit microsecond durations.
  // Saturating is safer than wrapping if the signal has a very long quiet gap.
  captureBuf[index] = duration > 65535UL ? 65535U : (uint16_t)duration;
  captureIsrLen = index + 1;
  captureEdgeUs = now;
  if (captureIsrLen >= CAPTURE_MAX_PULSES) {
    captureBufferFull = true;
    captureRecording = false;
  }
}

static bool captureIsReplayable(const uint16_t* timings, size_t len) {
  // A raw recording can legitimately contain a single mark or space. Do not
  // discard it merely because it is shorter than a typical remote packet.
  if (!timings || len < 1 || len > CAPTURE_MAX_PULSES) return false;
  for (size_t i = 0; i < len; i++) {
    // Zero-length timings can be produced by a malformed .sub file and make
    // its mark/space alignment meaningless. Reject it before touching TX.
    if (timings[i] == 0) return false;
  }
  return true;
}

static bool saveCaptureToFlash() {
  if (!captureHasData || !captureIsReplayable(captureBuf, captureLen)) return false;
  File f = LittleFS.open(CAPTURE_FILE, "w");   // "w" creates or truncates -> always one file
  if (!f) return false;
  f.println("Filetype: Flipper SubGhz RAW File");
  f.println("Version: 1");
  f.printf("Frequency: %lu\n", (unsigned long)(captureFreqMHz * 1000000.0f + 0.5f));
  f.println("Preset: FuriHalSubGhzPresetOok650Async");
  f.println("Protocol: RAW");
  for (size_t i = 0; i < captureLen; i++) {
    // Flipper's RAW format permits at most 512 timings per RAW_Data line.
    if ((i % 512) == 0) f.print("RAW_Data:");
    long v = captureBuf[i];
    bool mark = ((i % 2) == 0) ? captureStartsMark : !captureStartsMark;
    if (!mark) v = -v;                 // RAW_Data signs represent carrier on/off
    f.printf(" %ld", v);
    if ((i % 512) == 511 || i + 1 == captureLen) f.println();
  }
  f.close();
  return true;
}

static bool loadCaptureFromFlash() {
  File f = LittleFS.open(CAPTURE_FILE, "r");
  if (!f) return false;
  size_t n = 0;
  float freqMHz = CAPTURE_FREQ_MHZ;
  bool sawRaw = false, parseOk = true;
  bool startsMark = true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.startsWith("Frequency:")) {
      long hz = line.substring(10).toInt();
      if (hz > 0) freqMHz = hz / 1000000.0f;
    } else if (line.startsWith("RAW_Data:")) {
      sawRaw = true;
      String rest = line.substring(9);
      int pos = 0, len = rest.length();
      while (pos < len && n < CAPTURE_MAX_PULSES) {
        while (pos < len && rest[pos] == ' ') pos++;
        int start = pos;
        while (pos < len && rest[pos] != ' ') pos++;
        if (pos > start) {
          long raw = rest.substring(start, pos).toInt();
          bool mark = raw > 0;
          if (raw == 0 || raw < -65535L || raw > 65535L ||
              (n > 0 && mark == (((n % 2) == 0) ? startsMark : !startsMark))) {
            parseOk = false;
            break;
          }
          if (n == 0) startsMark = mark;
          captureBuf[n++] = (uint16_t)(raw < 0 ? -raw : raw);
        }
      }
      while (pos < len && rest[pos] == ' ') pos++;
      if (pos < len) parseOk = false;  // this sketch's fixed RAM capture is full
    }
  }
  f.close();
  if (!sawRaw || !parseOk || !captureIsReplayable(captureBuf, n)) return false;
  captureLen = n;
  captureHasData = true;
  captureLastFailed = false;
  captureFreqMHz = freqMHz;
  captureStartsMark = startsMark;
  captureBufferFull = false;
  return true;
}

static void showCaptureIdle() {
  beginFrame("CAPTURE");
  auto& d = M5.Display;
  d.setTextDatum(middle_center);
  int midX = (contentLeft + contentRight) / 2, midY = (contentTop + contentBottom) / 2;
  d.setTextSize(2);
  if (captureHasData) {
    d.setTextColor(captureBufferFull ? TFT_YELLOW : TFT_GREEN, TFT_BLACK);
    d.drawString(captureBufferFull ? "Buffer full" : "Captured", midX, midY - 10);
  } else if (captureLastFailed) {
    d.setTextColor(TFT_RED, TFT_BLACK);
    d.drawString("No signal", midX, midY - 10);
  } else {
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.drawString("No capture", midX, midY - 10);
  }
  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.setTextSize(1);
  char buf[32];
  if (captureHasData) snprintf(buf, sizeof(buf), "%u pulses @ %.2fMHz", (unsigned)captureLen, (double)captureFreqMHz);
  else                 snprintf(buf, sizeof(buf), "Listens at %.2fMHz", (double)CAPTURE_FREQ_MHZ);
  d.drawString(buf, midX, midY + 12);
  d.setTextColor(TFT_CYAN, TFT_BLACK);
  d.setTextDatum(bottom_center);
  d.drawString(captureHasData ? "Hold 4s: Save (flash)" : "Hold 4s: Load (flash)",
               midX, contentBottom - 2);
  drawHint("Short:REC", captureHasData ? "Med:Replay" : "");
}

static void showCaptureListening() {
  beginFrame("CAPTURE");
  auto& d = M5.Display;
  d.setTextColor(TFT_YELLOW, TFT_BLACK);
  d.setTextDatum(middle_center);
  d.setTextSize(2);
  d.drawString("Listening...", (contentLeft + contentRight) / 2, (contentTop + contentBottom) / 2 - 10);
  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.setTextSize(1);
  d.drawString("Press remote; short=Stop", (contentLeft + contentRight) / 2, (contentTop + contentBottom) / 2 + 12);
  drawHint("REC", "No time limit");
}

static void startCapture() {
  if (!radioOk) {
    radioOk = radioInit();
    if (!radioOk) { showNotFound(); return; }
  }

  // This intentionally records until the user presses Stop or the fixed RAM
  // buffer fills, matching Read RAW rather than stopping after one quiet gap.
  detachInterrupt(digitalPinToInterrupt(PIN_GDO0));
  ccSetFreqMHz(CAPTURE_FREQ_MHZ);
  ccStrobe(0x36);                           // SIDLE
  ccStrobe(0x34);                           // SRX
  pinMode(PIN_GDO0, INPUT);
  delayMicroseconds(500);

  noInterrupts();
  captureLen = 0;
  captureHasData = false;
  captureLastFailed = false;
  captureFreqMHz = CAPTURE_FREQ_MHZ;
  captureStartsMark = true;
  captureIsrLen = 0;
  captureStarted = false;
  captureBufferFull = false;
  captureRecording = true;
  interrupts();
  attachInterrupt(digitalPinToInterrupt(PIN_GDO0), onCaptureEdge, CHANGE);
  captureInterruptAttached = true;
  showCaptureListening();
}

static void stopCapture() {
  if (!captureInterruptAttached) return;
  captureRecording = false;
  detachInterrupt(digitalPinToInterrupt(PIN_GDO0));
  captureInterruptAttached = false;
  noInterrupts();
  captureLen = captureIsrLen;
  captureStartsMark = captureIsrStartsMark;
  interrupts();
  ccStrobe(0x36);                           // SIDLE
  ccSetFreqMHz(FREQ_MHZ);                   // park on the Tesla frequency

  captureHasData = captureStarted && captureIsReplayable(captureBuf, captureLen);
  captureLastFailed = !captureHasData;
  beep(captureHasData ? 1800 : 220, captureHasData ? 120 : 200);
  showCaptureIdle();
}

static void serviceCapture() {
  if (captureInterruptAttached && captureBufferFull) stopCapture();
}

static void replayCaptured() {
  if (!captureHasData || !captureIsReplayable(captureBuf, captureLen)) {
    captureLastFailed = true;
    beep(220, 200);
    showCaptureIdle();
    return;
  }
  if (!radioOk) {
    radioOk = radioInit();
    if (!radioOk) { showNotFound(); return; }
  }
  beginFrame("CAPTURE");
  auto& d = M5.Display;
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextDatum(middle_center);
  d.setTextSize(2);
  d.drawString("Replaying...", (contentLeft + contentRight) / 2, (contentTop + contentBottom) / 2);
  drawHint("", "");
  if (!ccTxStart(captureFreqMHz)) { beep(220, 200); ccTxStop(); showCaptureIdle(); return; }
  // RAW recordings already contain every repeat and inter-frame gap that was
  // captured. Send the recording once, just as Flipper's Saved RAW playback.
  sendBurst(captureBuf, captureLen, captureStartsMark);
  ccTxStop();
  beep(1800, 120);
  showCaptureIdle();
}

// M5StickS3's physical side button is BtnB. One press starts an endless RAW
// replay loop; the next press stops it. This never interrupts Read RAW capture.
static void serviceSideReplay() {
  if (!sideReplayRequested) {
    if (!sideReplayActive) return;
    ccTxStop();
    sideReplayActive = false;
    enterMode(mode);
    return;
  }

  if (captureInterruptAttached) return;
  if (!captureHasData || !captureIsReplayable(captureBuf, captureLen)) {
    sideReplayRequested = false;
    beep(220, 120);
    return;
  }

  if (!sideReplayActive) {
    if (!radioOk) {
      radioOk = radioInit();
      if (!radioOk) { showNotFound(); return; }
    }
    beginFrame("REPLAY");
    auto& d = M5.Display;
    d.setTextColor(TFT_YELLOW, TFT_BLACK);
    d.setTextDatum(middle_center);
    d.setTextSize(2);
    d.drawString("Side: Looping", (contentLeft + contentRight) / 2, (contentTop + contentBottom) / 2);
    drawHint("Repeating forever", "Side: Stop");
    sideReplayActive = true;
  }

  // Explicitly restart TX for every pass. This guarantees a new transmission
  // even when the CC1101 has returned to IDLE after the previous RAW stream.
  if (!ccTxStart(captureFreqMHz)) {
    beep(220, 200);
    ccTxStop();
    sideReplayActive = false;
    sideReplayRequested = false;
    return;
  }
  sendBurst(captureBuf, captureLen, captureStartsMark);
  ccTxStop();
  delayMicroseconds(GAP_US);
}

static void enterMode(Mode m) {
  if (captureInterruptAttached && m != MODE_CAPTURE) stopCapture();
  mode = m;
  switch (mode) {
    case MODE_TESLA:
      digitalWrite(PIN_NRF_CS, HIGH); digitalWrite(PIN_NRF_CE, LOW);
      if (radioOk) showReady(); else showNotFound();
      break;
    case MODE_SUBGHZ:
      digitalWrite(PIN_NRF_CS, HIGH); digitalWrite(PIN_NRF_CE, LOW);
      subghzPeakInit = false;
      beginFrame("SUBGHZ"); drawHint("Scanning...", "Med:Reset peak");
      break;
    case MODE_NRF:
      digitalWrite(PIN_CSN, HIGH);
      nrfInit();
      beginFrame("NRF24"); drawHint("Scanning...", "Med:Fast/Thorough");
      break;
    case MODE_CAPTURE:
      digitalWrite(PIN_NRF_CS, HIGH); digitalWrite(PIN_NRF_CE, LOW);
      // Internal flash, not an SD card, so this is a fast local check - safe to
      // do automatically (unlike the old SD version, there's no bus to hang on).
      if (!captureHasData) loadCaptureFromFlash();
      showCaptureIdle();
      break;
    default: break;
  }
}

// feature 2: boot self-test for both radios before the main UI loads
static void bootSelfTest() {
  auto& d = M5.Display;
  d.fillScreen(TFT_BLACK);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextDatum(top_left);
  d.setTextSize(1);
  d.drawString("StickS3 RF Tool - self test", 4, 4);

  radioOk = radioInit();
  ccPresent = radioOk;
  char line1[36];
  snprintf(line1, sizeof(line1), "CC1101  : %s (V=0x%02X)", ccPresent ? "OK" : "FAIL", lastVersion);
  d.setTextColor(ccPresent ? TFT_GREEN : TFT_RED, TFT_BLACK);
  d.drawString(line1, 4, 20);

  digitalWrite(PIN_CSN, HIGH);
  nrfInit();
  nrfPresent = nrfDetect();
  char line2[24];
  snprintf(line2, sizeof(line2), "nRF24L01: %s", nrfPresent ? "OK" : "FAIL");
  d.setTextColor(nrfPresent ? TFT_GREEN : TFT_RED, TFT_BLACK);
  d.drawString(line2, 4, 32);

  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.drawString("starting...", 4, 48);
  delay(900);
}

// ================== Button (3-tier) + auto-sleep ==================
static uint32_t pressStart = 0;
static bool     wasDown = false;
static uint32_t lastActivity = 0;
static bool     sleeping = false;

static void wake() {
  if (sleeping) { M5.Display.setBrightness(255); sleeping = false; enterMode(mode); }
  lastActivity = millis();
}

static void onShortPress() {
  if (mode == MODE_TESLA) {
    if (!radioOk) {
      radioOk = radioInit();
      if (!radioOk) { showNotFound(); return; }
    }
    openChargePort();
    delay(1200);
    if (radioOk) showReady();
  }
  else if (mode == MODE_CAPTURE) {
    if (captureInterruptAttached) stopCapture();
    else                          startCapture();
  }
  // SUBGHZ / NRF: short press has no action, the scan free-runs every loop
}

static void onMediumPress() {
  if (mode == MODE_CAPTURE && captureInterruptAttached) { stopCapture(); return; }
  switch (mode) {
    case MODE_TESLA:  cycleTxPower();        break;   // feature 5
    case MODE_SUBGHZ: subghzPeakInit = false; beep(1200, 60); break;   // feature 7 reset
    case MODE_NRF:     toggleNrfScanSpeed();  break;   // feature 10
    case MODE_CAPTURE: replayCaptured();      break;
    default: break;
  }
}

static void onLongPress() {
  if (mode == MODE_CAPTURE && captureInterruptAttached) stopCapture();
  beep(900, 80);
  enterMode((Mode)((mode + 1) % MODE_COUNT));
}

// Fourth tier (>4s), CAPTURE mode only: save the current capture to the single
// /capture.sub file on internal flash (overwriting it), or if nothing is
// captured yet, try loading it back. Every other mode treats a very long hold
// the same as the normal next-mode hold.
static void onVeryLongPress() {
  if (mode == MODE_CAPTURE && captureInterruptAttached) { stopCapture(); return; }
  if (mode != MODE_CAPTURE) { onLongPress(); return; }

  bool saving = captureHasData;
  beginFrame("CAPTURE");
  auto& d = M5.Display;
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextDatum(middle_center);
  d.setTextSize(2);
  d.drawString(saving ? "Saving..." : "Loading...", (contentLeft + contentRight) / 2, (contentTop + contentBottom) / 2);
  drawHint("", "");

  bool ok = saving ? saveCaptureToFlash() : loadCaptureFromFlash();
  beep(ok ? 1800 : 220, ok ? 120 : 200);

  if (!ok) {
    beginFrame("CAPTURE");
    d.setTextColor(TFT_RED, TFT_BLACK);
    d.setTextSize(1);
    d.drawString(saving ? "Flash save failed" : "No saved capture in flash",
                 (contentLeft + contentRight) / 2, (contentTop + contentBottom) / 2);
    drawHint("", "");
    delay(1200);
  }
  showCaptureIdle();
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  M5.Display.setRotation(1);
  if (!LittleFS.begin(true)) {    // true = format internal flash if no filesystem is found yet
    Serial.println("LittleFS mount failed - capture save/load will be unavailable");
  }
  bootSelfTest();                 // feature 2
  lastActivity = millis();
  enterMode(MODE_TESLA);
}

void loop() {
  M5.update();

  if (M5.BtnA.isPressed()) {
    if (!wasDown) { wasDown = true; pressStart = millis(); }
  }
  if (M5.BtnA.wasReleased()) {
    wasDown = false;
    wake();
    uint32_t held = millis() - pressStart;
    beep(2000, 25);                               // feature 4: click feedback
    if (held < 500)        onShortPress();
    else if (held < 1500)  onMediumPress();
    else if (held < 4000)  onLongPress();
    else                   onVeryLongPress();
  }
  if (M5.BtnA.isPressed()) wake();                 // any touch resets the idle timer early
  if (M5.BtnB.wasPressed()) {
    wake();
    if (sideReplayRequested || sideReplayActive) {
      sideReplayRequested = false;
    } else if (!captureInterruptAttached && captureHasData && captureIsReplayable(captureBuf, captureLen)) {
      sideReplayRequested = true;
    } else {
      beep(220, 120);                              // no completed RAW recording yet
    }
  }

  // feature 11: auto screen-sleep after IDLE_SLEEP_MS with no button activity
  if (!sleeping && !captureInterruptAttached && !sideReplayActive && millis() - lastActivity > IDLE_SLEEP_MS) {
    sleeping = true;
    M5.Display.setBrightness(0);
  }

  serviceCapture();
  serviceSideReplay();
  if (!sleeping) {
    // Spectrum scans retune the CC1101, so do not let them interrupt an active
    // side-button RAW replay between its consecutive transmission passes.
    if (!sideReplayActive && mode == MODE_SUBGHZ)      runSubghzFrame();
    else if (!sideReplayActive && mode == MODE_NRF)    runNrfFrame();
    else if (mode == MODE_TESLA)  { /* static screen, redrawn on events only (battery/dots still age) */ }
  }

  delay(5);
}
