#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <rtl_433_ESP.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <math.h>
#include <string.h>

// ============================================================
// HARDWARE CONNECTIONS (SX1278 to Arduino Nano ESP32)
// ============================================================
//
// SX1278:
// VCC  = 3.3V
// GND  = GND
// NSS  = D10  (SPI CS)
// SCK  = D13  (SPI SCK)
// MOSI = D11  (SPI MOSI)
// MISO = D12  (SPI MISO)
// DIO0 = A1   (Optional IRQ line)
// DIO1 = A2   (Optional IRQ line)
// DIO2 = D2   (FSK Data demodulation output)
// RST  = A3   (Hardware Reset)
//
// OLED:
// SDA  = A4
// SCL  = A5
//
// ULN2803A Dashboard Drivers:
// D3   = Dashboard low-pressure warning
// D4   = FL LED
// D5   = FR LED
// D6   = RR LED
// D7   = RL LED
// D8   = Active buzzer
// A0   = Dashboard sensor-fault warning
//
// Mute Button:
// D9   = Pushbutton to GND
//
// ============================================================

const uint8_t PIN_LOW_WARNING  = D3;
const uint8_t PIN_BUZZER       = D8;
const uint8_t PIN_MUTE         = D9;
const uint8_t PIN_SENSOR_FAULT = A0;

const uint8_t wheelLEDs[4] = { D4, D5, D6, D7 };

const uint8_t OLED_ADDRESS = 0x3C;

// ============================================================
// TIMING & ALARM CONSTANTS
// ============================================================

const uint32_t STARTUP_GRACE_MS  = 10UL * 60UL * 1000UL; // 10 minutes
const uint32_t SENSOR_STALE_MS   = 15UL * 60UL * 1000UL; // 15 minutes
const uint32_t SOUNDER_SETTLE_MS = 20000UL;              // 20 seconds initial quiet
const float HYSTERESIS_PSI       = 1.0f;

// ============================================================
// DATA STRUCTURES
// ============================================================

struct WheelState {
  uint32_t id;
  const char* name;
  float alarmPSI;

  float pressurePSI;
  float temperatureC;

  uint32_t lastSeen;

  bool received;
  bool low;
};

struct AlarmState {
  bool lowPressure;
  bool sensorFault;
  bool soundAllowed;
};

struct RadioMessage {
  char text[1024];
};

// Wheel array order: FL, FR, RR, RL.
WheelState wheels[4] = {
  { 0x1C604B01UL, "FL", 30.0f, 0.0f, 0.0f, 0UL, false, false },
  { 0x1C4BDB04UL, "FR", 30.0f, 0.0f, 0.0f, 0UL, false, false },
  { 0x1C604AC1UL, "RR", 32.0f, 0.0f, 0.0f, 0UL, false, false },
  { 0x1C6050DFUL, "RL", 32.0f, 0.0f, 0.0f, 0UL, false, false }
};

// ============================================================
// GLOBAL OBJECTS
// ============================================================

rtl_433_ESP receiver;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

char decodeBuffer[2048];
QueueHandle_t radioQueue = nullptr;

uint32_t bootTime = 0;
bool receiverOK   = false;
bool oledOK       = false;
bool muted        = false;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void onDecoded(char* message);
void processMessage(const char* message, uint32_t now);
bool sensorMissing(const WheelState& wheel, uint32_t now);
AlarmState getAlarms(uint32_t now);
void updateMute(uint32_t now);
void updateOutputs(uint32_t now, const AlarmState& alarms);
void updateDisplay(uint32_t now, const AlarmState& alarms);

// ============================================================
// RECEIVER CALLBACK
// ============================================================

void onDecoded(char* message) {
  if (message == nullptr || radioQueue == nullptr) {
    return;
  }

  const size_t length = strlen(message);
  RadioMessage incoming;

  if (length >= sizeof(incoming.text)) {
    return;
  }

  memcpy(incoming.text, message, length + 1);
  xQueueSend(radioQueue, &incoming, 0);
}

// ============================================================
// PROCESS SENSOR MESSAGE
// ============================================================

void processMessage(const char* message, uint32_t now) {
  Serial.print("DECODED RAW: ");
  Serial.println(message);

  JsonDocument document;
  DeserializationError error = deserializeJson(document, message);

  if (error) {
    Serial.print("JSON Error: ");
    Serial.println(error.c_str());
    return;
  }

  const char* model = document["model"] | "";
  const char* mic   = document["mic"] | "";

  // Sensor model identified for Aston Martin AM305 sensors
  if (strcmp(model, "BMW-GEN2") != 0) {
    return;
  }

  if (strcmp(mic, "CRC") != 0) {
    return;
  }

  if (!document["id"].is<uint32_t>() ||
      !document["pressure_kPa"].is<float>() ||
      !document["temperature_C"].is<float>()) {
    Serial.println("Missing required fields in payload");
    return;
  }

  const uint32_t id        = document["id"].as<uint32_t>();
  const float pressureKPa  = document["pressure_kPa"].as<float>();
  const float temperatureC = document["temperature_C"].as<float>();

  if (!isfinite(pressureKPa) ||
      !isfinite(temperatureC) ||
      pressureKPa < 0.0f ||
      pressureKPa > 1000.0f ||
      temperatureC < -50.0f ||
      temperatureC > 150.0f) {
    Serial.println("Sensor values out of physical bounds");
    return;
  }

  const float pressurePSI = pressureKPa * 0.1450377377f;

  for (int i = 0; i < 4; i++) {
    if (wheels[i].id != id) {
      continue;
    }

    wheels[i].pressurePSI  = pressurePSI;
    wheels[i].temperatureC = temperatureC;
    wheels[i].lastSeen     = now;
    wheels[i].received     = true;

    if (pressurePSI < wheels[i].alarmPSI) {
      wheels[i].low = true;
    } else if (pressurePSI >= wheels[i].alarmPSI + HYSTERESIS_PSI) {
      wheels[i].low = false;
    }

    Serial.print(wheels[i].name);
    Serial.print(" ID=");
    Serial.print(id, HEX);
    Serial.print(" Pressure=");
    Serial.print(pressurePSI, 1);
    Serial.print(" PSI | Temp=");
    Serial.print(temperatureC, 1);
    Serial.print(" C | ");
    Serial.println(wheels[i].low ? "LOW ALARM" : "OK");

    return;
  }

  Serial.print("Unassigned sensor ID: 0x");
  Serial.println(id, HEX);
}

// ============================================================
// SENSOR TIMEOUT CALCULATION
// ============================================================

bool sensorMissing(const WheelState& wheel, uint32_t now) {
  if (!wheel.received) {
    return uint32_t(now - bootTime) >= STARTUP_GRACE_MS;
  }

  return uint32_t(now - wheel.lastSeen) >= SENSOR_STALE_MS;
}

// ============================================================
// ALARM LOGIC
// ============================================================

AlarmState getAlarms(uint32_t now) {
  AlarmState alarms = { false, false, false };
  bool allReceived  = true;

  for (int i = 0; i < 4; i++) {
    if (wheels[i].low) {
      alarms.lowPressure = true;
    }

    if (sensorMissing(wheels[i], now)) {
      alarms.sensorFault = true;
    }

    if (!wheels[i].received) {
      allReceived = false;
    }
  }

  if (!receiverOK) {
    alarms.sensorFault = true;
  }

  const uint32_t uptime = uint32_t(now - bootTime);

  bool settled =
    uptime >= SOUNDER_SETTLE_MS &&
    (allReceived || uptime >= STARTUP_GRACE_MS);

  if (!receiverOK) {
    settled = uptime >= SOUNDER_SETTLE_MS;
  }

  alarms.soundAllowed =
    !muted &&
    settled &&
    (alarms.lowPressure || alarms.sensorFault);

  return alarms;
}

// ============================================================
// MUTE DEBOUNCE
// ============================================================

void updateMute(uint32_t now) {
  static bool lastReading   = HIGH;
  static bool stableReading = HIGH;
  static uint32_t changedAt = 0;

  const bool reading = digitalRead(PIN_MUTE);

  if (reading != lastReading) {
    lastReading = reading;
    changedAt   = now;
  }

  if (uint32_t(now - changedAt) >= 40UL && reading != stableReading) {
    stableReading = reading;

    if (stableReading == LOW && !muted) {
      muted = true;
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println("Sounder muted until ignition restart");
    }
  }
}

// ============================================================
// HARDWARE OUTPUTS
// ============================================================

void updateOutputs(uint32_t now, const AlarmState& alarms) {
  const bool flashOn = ((now / 500UL) % 2UL) == 0;

  for (int i = 0; i < 4; i++) {
    bool ledOn = false;

    if (wheels[i].low) {
      ledOn = true;
    } else if (!receiverOK || !wheels[i].received || sensorMissing(wheels[i], now)) {
      ledOn = flashOn;
    }

    digitalWrite(wheelLEDs[i], ledOn ? HIGH : LOW);
  }

  digitalWrite(PIN_LOW_WARNING, alarms.lowPressure ? HIGH : LOW);
  digitalWrite(PIN_SENSOR_FAULT, alarms.sensorFault ? HIGH : LOW);

  const bool buzzerOn = alarms.soundAllowed && (now % 1000UL < 250UL);
  digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
}

// ============================================================
// OLED DISPLAY
// ============================================================

void updateDisplay(uint32_t now, const AlarmState& alarms) {
  if (!oledOK) {
    return;
  }

  display.clearDisplay();
  display.setFont(nullptr);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.setTextSize(1);

  display.setCursor(0, 3);
  display.print("ASTON TPMS");

  display.setCursor(66, 3);
  display.print("PSI");

  if (muted) {
    display.setCursor(96, 3);
    display.print("MUTE");
  }

  const uint8_t screenOrder[4] = { 0, 1, 3, 2 };

  for (int cell = 0; cell < 4; cell++) {
    const int i = screenOrder[cell];
    const int x = (cell % 2) * 64;
    const int y = (cell < 2) ? 18 : 38;

    const bool missing = !receiverOK || sensorMissing(wheels[i], now);

    display.setTextSize(1);
    display.setCursor(x, y);
    display.print(wheels[i].name);

    display.setCursor(x, y + 9);
    if (missing) {
      display.print("?");
    } else if (!wheels[i].received) {
      display.print("W");
    } else if (wheels[i].low) {
      display.print("LO");
    } else {
      display.print("OK");
    }

    display.setTextSize(2);
    display.setCursor(x + 16, y);

    if (!wheels[i].received || missing) {
      display.print("--.-");
    } else if (wheels[i].pressurePSI >= 100.0f) {
      display.print(wheels[i].pressurePSI, 0);
    } else {
      display.print(wheels[i].pressurePSI, 1);
    }
  }

  display.setTextSize(1);
  display.setCursor(0, 56);

  if (!receiverOK) {
    display.print("RECEIVER FAULT");
  } else if (alarms.lowPressure && alarms.sensorFault) {
    display.print("LOW PSI + SENSOR FAULT");
  } else if (alarms.lowPressure) {
    display.print("CHECK TYRES");
  } else if (alarms.sensorFault) {
    display.print("SENSOR FAULT");
  } else {
    bool waiting = false;
    for (int i = 0; i < 4; i++) {
      if (!wheels[i].received) {
        waiting = true;
      }
    }
    display.print(waiting ? "WAITING FOR SENSORS" : "ALL TYRES OK");
  }

  display.display();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  const uint8_t outputPins[] = {
    PIN_LOW_WARNING, D4, D5, D6, D7, PIN_BUZZER, PIN_SENSOR_FAULT
  };

  for (uint8_t pin : outputPins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }

  pinMode(PIN_MUTE, INPUT_PULLUP);

  bootTime = millis();
  Serial.begin(115200);

  const uint32_t serialStarted = millis();
  while (!Serial && uint32_t(millis() - serialStarted) < 4000UL) {
    delay(10);
  }

  Serial.println();
  Serial.println("Aston Martin TPMS");
  Serial.println("SX1278 FSK - 433.92 MHz");
  Serial.println("Front alarm below 30 PSI");
  Serial.println("Rear alarm below 32 PSI");

  Serial.println("Expected sensor IDs:");
  Serial.println("FL: 1C604B01");
  Serial.println("FR: 1C4BDB04");
  Serial.println("RR: 1C604AC1");
  Serial.println("RL: 1C6050DF");

  Wire.begin(A4, A5);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() == 0) {
    oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, true, false);
  }

  Serial.print("OLED: ");
  Serial.println(oledOK ? "READY" : "FAILED");

  if (oledOK) {
    display.setRotation(2);
    display.clearDisplay();
    display.setFont(nullptr);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.setCursor(0, 3);
    display.print("ASTON TPMS");
    display.setCursor(0, 24);
    display.print("Starting SX1278...");
    display.display();
  }

  radioQueue = xQueueCreate(8, sizeof(RadioMessage));
  if (radioQueue == nullptr) {
    Serial.println("Message queue allocation FAILED");
  } else {
    receiver.setCallback(onDecoded, decodeBuffer, sizeof(decodeBuffer));
    receiverOK = receiver.begin(D2, 433.92f);

    if (receiverOK) {
      receiver.enableReceiver();
      receiver.getModuleStatus();
    } else {
      Serial.println("SX1278 receiver initialization FAILED");
    }
  }

  Serial.print("Receiver: ");
  Serial.println(receiverOK ? "READY" : "FAILED");
  Serial.println("Monitoring TPMS sensors");

  const uint32_t now = millis();
  const AlarmState alarms = getAlarms(now);
  updateOutputs(now, alarms);
  updateDisplay(now, alarms);
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  const uint32_t now = millis();
  updateMute(now);

  if (receiverOK) {
    receiver.loop();
  }

  if (radioQueue != nullptr) {
    RadioMessage incoming;
    for (int count = 0; count < 8; count++) {
      if (xQueueReceive(radioQueue, &incoming, 0) != pdTRUE) {
        break;
      }
      processMessage(incoming.text, millis());
    }
  }

  const uint32_t updateTime = millis();
  const AlarmState alarms = getAlarms(updateTime);
  updateOutputs(updateTime, alarms);
  updateDisplay(updateTime, alarms);
    static uint32_t lastDiagnostics = 0;

  if (uint32_t(millis() - lastDiagnostics) >= 5000UL) {
    lastDiagnostics = millis();

    Serial.printf(
      "RX DIAG: captures=%u decoded=%u unsuccessful=%u "
      "captureDrops=%u queueDrops=%u\n",
      (unsigned int)rtl_433_ESP::decoderSignals,
      (unsigned int)rtl_433_ESP::decodedMessages,
      (unsigned int)rtl_433_ESP::zeroDecodedSignals,
      (unsigned int)rtl_433_ESP::droppedCaptureBuffers,
      (unsigned int)rtl_433_ESP::droppedDecoderQueue
    );
  }
  delay(10);
}
