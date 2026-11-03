# TPMS

Arduino Nano ESP32 with an SX1278 Ra-02 receiver and SSD1306 OLED.
Open this repository root as the PlatformIO project. The configuration is
`platformio.ini`, the application is `src/main.cpp`, and the customized receiver
library is bundled under `lib/rtl_433_ESP`.

Build with `pio run`, upload with `pio run --target upload`, and open the serial
monitor with `pio device monitor` (115200 baud).

For missing readings, capture the startup output and at least 30 seconds of
`RX DIAG` and status messages while sensors are transmitting. Check that the
receiver reports READY and FSK modulation. Capture/decoder counts and RSSI help
distinguish reception problems from messages rejected by the application.
`DECODED RAW` shows decoder output; only CRC-checked BMW-GEN2 messages matching
the four configured wheel IDs update the display. Other models and unassigned
IDs are logged. SX1278 DIO2 must connect to D2 as shown in `src/main.cpp`.
