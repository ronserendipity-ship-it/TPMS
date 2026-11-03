# CC1101 TPMS bench transmitter

Open this folder as the PlatformIO project, rather than the receiver repository
root. The tester application is `src/main.cpp`; its configuration is the
`platformio.ini` in this folder.

Build from this folder with `pio run`, or upload with `pio run --target upload`.
The log must identify `nano_esp32_cc1101_tester`.

Use a separate Arduino Nano ESP32 connected to the CC1101. To identify the target
unambiguously during DFU upload, disconnect the receiver Nano from USB first.

Open the tester's serial monitor at 115200 baud. After reset, expect
`BMW Gen2 TPMS bench transmitter v2`, `CC1101 init status: 0`, and then `alive`.
Send `a` followed by a newline to transmit all four configured wheels. Send
`r 5` for automatic transmission every five seconds, or `r 0` to stop it.

Wiring: VCC to 3.3V, GND to GND, CS to D10, SCK to D13, MOSI to D11,
MISO to D12, and GDO0 to D2. Leave GDO2 unconnected.
