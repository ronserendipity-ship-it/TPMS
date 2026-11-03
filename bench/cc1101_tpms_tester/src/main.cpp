// ============================================================
// BMW GEN2 TPMS BENCH TRANSMITTER (CC1101, RadioLib)  v2
// ============================================================
// Fakes the four wheel sensors so you can test the receiver
// on the bench. Use a SECOND board for this; the receiver
// board cannot also be the transmitter.
//
// Frame format, from rtl_433 src/devices/tpms_bmw_g3.c:
//   433.92 MHz, FSK, PCM, 52 us per raw bit
//   preamble 0xCCCD, then differential Manchester payload
//   payload (Gen2, 10 bytes + 1 trailing bit):
//     [ID3 ID2 ID1 ID0 PP TT F1 F2] CRC_HI CRC_LO  0
//   PP: kPa = (PP - 43) * 2.5       (PSI = (PP - 43) * 0.363)
//   TT: temperature C = TT - 40
//   CRC: CRC-16, poly 0x1021, init 0x0000, over the first 8 bytes
//        (verified against the three sample frames in the source)
//
// WIRING (CC1101 to this board), change the pins below to match:
//   VCC 3.3V, GND, CS=D10, SCK=D13, MOSI=D11, MISO=D12,
//   GDO0=D2 (data INPUT to the radio in async TX mode), GDO2=unused
//
// SERIAL COMMANDS (115200 baud, send with newline):
//   w <0-3>    select wheel: 0=FL 1=FR 2=RR 3=RL
//   p <psi>    set pressure in PSI for the selected wheel
//   t <C>      set temperature in C for the selected wheel
//   s          send one frame for the selected wheel now
//   a          send all four wheels once
//   r <secs>   auto-repeat all four wheels every N seconds (0 = off)
//   pw <dBm>   TX power: -30 -20 -15 -10 0 5 7 10
//   h          help / show state
//
// Use a RadioLib version exposing transmitDirectAsync() (checked with 7.8.1).
// For Nano ESP32 builds, use BOARD_USES_HW_GPIO_NUMBERS consistently
// across the application and RadioLib, as in the receiver project.
// Keep the default line coding for the initial test: it matches the
// bundled rtl_433 differential Manchester decoder.
//
// SERIAL STATUS:
//   "alive" prints every 3 seconds so you can always see the
//   sketch is running. If the CC1101 is not found, the error
//   repeats every 2 seconds instead of stopping silently.
// ============================================================

#include <Arduino.h>
#include <RadioLib.h>
#include <math.h>

// ---------------- pins ----------------
// D-style names match the receiver sketch (Nano ESP32).
// If your board does not define D10/D2/D3, use plain numbers.
const int PIN_CS   = D10;
const int PIN_GDO0 = D2;   // data in
const int PIN_GDO2 = D3;   // unused but passed to RadioLib

// ---------------- radio settings ----------------
const float FREQ_MHZ     = 433.92f;
const float FSK_DEV_KHZ  = 35.0f;   // tune if needed
const float RX_BW_KHZ    = 58.0f;   // unused for TX, required by begin()
const float BITRATE_KBPS = 19.2f;   // unused in async mode, ~1/52us
int8_t txPowerDbm        = -10;     // start low for bench use

// ---------------- line coding switches ----------------
#define DM_ONE_IS_MID_TRANSITION 0  // 0: data 1 = no mid-bit transition
#define INVERT_OUTPUT            0  // 1: invert every raw bit sent

const uint32_t HALF_BIT_US = 52;
const int LEADIN_BITS = 16;         // alternating 1010... before preamble
const uint16_t PREAMBLE = 0xCCCD;

CC1101 radio = new Module(PIN_CS, PIN_GDO0, RADIOLIB_NC, PIN_GDO2);

struct FakeWheel {
  const char* name;
  uint32_t id;
  float psi;
  float tempC;
};

// Order: FL, FR, RR, RL (same IDs as the receiver sketch)
FakeWheel wheels[4] = {
  {"FL", 0x1C604B01UL, 32.0f, 20.0f},
  {"FR", 0x1C4BDB04UL, 32.0f, 20.0f},
  {"RR", 0x1C604AC1UL, 34.0f, 20.0f},
  {"RL", 0x1C6050DFUL, 34.0f, 20.0f}
};

int selected = 0;
uint32_t repeatSecs = 0;
uint32_t lastRepeat = 0;

// ---------------- CRC-16 (poly 0x1021, init 0) ----------------
uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0x0000;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) {
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021)
                       : (uint16_t)(c << 1);
    }
  }
  return c;
}

// ---------------- build the 10-byte Gen2 payload ----------------
void buildPayload(const FakeWheel& w, uint8_t out[10]) {
  float kPa = w.psi / 0.1450377377f;
  int pp = (int)lroundf(kPa / 2.5f) + 43;
  int tt = (int)lroundf(w.tempC) + 40;
  pp = constrain(pp, 0, 255);
  tt = constrain(tt, 0, 255);

  out[0] = (w.id >> 24) & 0xFF;
  out[1] = (w.id >> 16) & 0xFF;
  out[2] = (w.id >> 8) & 0xFF;
  out[3] = w.id & 0xFF;
  out[4] = (uint8_t)pp;
  out[5] = (uint8_t)tt;
  out[6] = 0x80;   // flags1 (from sample frames)
  out[7] = 0x51;   // flags2 (from sample frames)

  uint16_t crc = crc16(out, 8);
  out[8] = crc >> 8;
  out[9] = crc & 0xFF;
}

// ---------------- raw half-bit stream ----------------
// 16 (lead-in) + 16 (preamble) + 2 * 81 (payload) = 194 max
const int MAX_RAW = 256;
uint8_t raw[MAX_RAW];
int rawLen = 0;

void pushRaw(uint8_t b) {
  if (rawLen < MAX_RAW) raw[rawLen++] = b ? 1 : 0;
}

void buildFrame(const uint8_t payload[10]) {
  rawLen = 0;

  // Lead-in: alternating bits
  for (int i = 0; i < LEADIN_BITS; i++) pushRaw((i & 1) == 0);

  // Preamble, MSB first
  for (int i = 15; i >= 0; i--) pushRaw((PREAMBLE >> i) & 1);

  // Differential Manchester payload, 80 bits + 1 trailing 0 bit.
  // Level continues from the last preamble bit (1).
  uint8_t level = PREAMBLE & 1;

  for (int bitIndex = 0; bitIndex < 81; bitIndex++) {
    uint8_t bit = 0;
    if (bitIndex < 80) {
      bit = (payload[bitIndex / 8] >> (7 - (bitIndex % 8))) & 1;
    }

    // clock transition at the start of every bit
    level ^= 1;
    pushRaw(level);

#if DM_ONE_IS_MID_TRANSITION
    if (bit) level ^= 1;
#else
    if (!bit) level ^= 1;
#endif
    pushRaw(level);
  }
}

// ---------------- transmit ----------------
void sendFrame(const FakeWheel& w) {
  uint8_t payload[10];
  buildPayload(w, payload);
  buildFrame(payload);

  Serial.print("TX ");
  Serial.print(w.name);
  Serial.print(" ID=");
  Serial.print(w.id, HEX);
  Serial.print(" PSI=");
  Serial.print(w.psi, 1);
  Serial.print(" T=");
  Serial.print(w.tempC, 0);
  Serial.print("C bytes=");
  for (int i = 0; i < 10; i++) {
    if (payload[i] < 16) Serial.print('0');
    Serial.print(payload[i], HEX);
  }
  Serial.println();
  Serial.flush();

  // Bit-banged data on GDO0 requires asynchronous direct mode.
  // transmitDirect() selects synchronous mode (GDO0 becomes a clock).
  const int16_t txStatus = radio.transmitDirectAsync();
  if (txStatus != RADIOLIB_ERR_NONE) {
    Serial.print("TX start failed, status ");
    Serial.println(txStatus);
    radio.standby();
    return;
  }
  pinMode(PIN_GDO0, OUTPUT);
  digitalWrite(PIN_GDO0, LOW);
  delay(2);   // let the PLL and PA settle

  noInterrupts();
  uint32_t next = micros();
  for (int i = 0; i < rawLen; i++) {
    uint8_t level = raw[i];
#if INVERT_OUTPUT
    level ^= 1;
#endif
    digitalWrite(PIN_GDO0, level);
    next += HALF_BIT_US;
    while ((int32_t)(micros() - next) < 0) { }
  }
  digitalWrite(PIN_GDO0, LOW);
  interrupts();

  radio.standby();
}

void sendAll() {
  for (int i = 0; i < 4; i++) {
    sendFrame(wheels[i]);
    // Real sensors send a burst; repeat each frame a few times
    delay(60);
  }
}

// ---------------- serial commands ----------------
void printState() {
  Serial.println("--- bench TX state ---");
  for (int i = 0; i < 4; i++) {
    Serial.print(i == selected ? "> " : "  ");
    Serial.print(wheels[i].name);
    Serial.print(" ID=");
    Serial.print(wheels[i].id, HEX);
    Serial.print(" PSI=");
    Serial.print(wheels[i].psi, 1);
    Serial.print(" T=");
    Serial.print(wheels[i].tempC, 0);
    Serial.println("C");
  }
  Serial.print("Power ");
  Serial.print(txPowerDbm);
  Serial.print(" dBm | repeat ");
  Serial.print(repeatSecs);
  Serial.println(" s");
  Serial.println("Commands: w p t s a r pw h");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("pw")) {
    txPowerDbm = (int8_t)line.substring(2).toInt();
    int16_t st = radio.setOutputPower(txPowerDbm);
    Serial.print("Power set, status ");
    Serial.println(st);
    return;
  }

  char c = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();

  switch (c) {
    case 'w': {
      int n = arg.toInt();
      if (n >= 0 && n < 4) selected = n;
      printState();
      break;
    }
    case 'p':
      wheels[selected].psi = arg.toFloat();
      sendFrame(wheels[selected]);
      break;
    case 't':
      wheels[selected].tempC = arg.toFloat();
      sendFrame(wheels[selected]);
      break;
    case 's':
      sendFrame(wheels[selected]);
      break;
    case 'a':
      sendAll();
      break;
    case 'r':
      repeatSecs = (uint32_t)arg.toInt();
      lastRepeat = millis();
      printState();
      break;
    default:
      printState();
      break;
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  pinMode(PIN_GDO0, OUTPUT);
  digitalWrite(PIN_GDO0, LOW);

  Serial.println();
  Serial.println("BMW Gen2 TPMS bench transmitter v2");
  Serial.println("Starting CC1101...");

  int16_t st = radio.begin(
    FREQ_MHZ, BITRATE_KBPS, FSK_DEV_KHZ, RX_BW_KHZ, txPowerDbm, 16);
  Serial.print("CC1101 init status: ");
  Serial.println(st);

  if (st != RADIOLIB_ERR_NONE) {
    // Repeat the error forever so it cannot be missed.
    while (true) {
      Serial.print("CC1101 not found, status ");
      Serial.print(st);
      Serial.println(" - check wiring and 3.3V supply");
      delay(2000);
    }
  }

  st = radio.setOOK(false);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("FSK configuration failed, status ");
    Serial.println(st);
    while (true) delay(1000);
  }
  // RadioLib initialization configures module pins; restore TX data output.
  pinMode(PIN_GDO0, OUTPUT);
  digitalWrite(PIN_GDO0, LOW);
  printState();
}

void loop() {
  // Heartbeat so you can always see the sketch is running.
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 3000) {
    lastBeat = millis();
    Serial.println("alive");
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }

  if (repeatSecs > 0 && millis() - lastRepeat >= repeatSecs * 1000UL) {
    lastRepeat = millis();
    sendAll();
  }
}
