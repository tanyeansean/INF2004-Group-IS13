# integrated_car — bare-metal integration test firmware

**Test harness, not the team firmware.** A plain Pico SDK build (no µT-Kernel) used on
2026-10-26 to test, on the real car:

- Buddy 3 `track_purist/` — IR line following (2 front sensors straddling the line), barcode
  decode, `nav_cmd` barcode-map handling. Compiled from `../track_purist`, not copied.
- Buddy 1 `robocar_comms` v0.3.1 — WiFi / MQTT / telemetry, copied into `comms/` (Aurelius owns it).
  Every MQTT payload is packed binary (Change Requirement 1); layouts in `comms/packets.h`.
- `motor.c` — open-loop PWM for the Robo Pico motors (M1 = GP8/9, M2 = GP10/11). No encoders/PID.

It calls lwIP MQTT directly, which the µT-Kernel template does not allow for student tasks, so it
stays outside `app_program/` and is not part of the `build_make` build.

## Behaviour
- Boot → calibrate front IR sensors (both on black, then both on white) → `READY`.
- `start` (MQTT) or `S` (serial): follow the line. Both white = straight; one black = gentle turn
  toward the line; both black (junction) = stop. `stop`/`estop` or `X` stops.
- Laptop map `map A B C D` → `nav_map_apply()` → ack `0` (verified on hardware).
- `speed N` (MQTT): comms caps N at 20 cm/s; `main.c` sets the base PWM to N × `PWM_AT_SPEED_MAX` / 20
  (85 until measured on the track). The ack shows the speed used.
- Tuning at the top of `main.c`: `BASE_PWM_M1/M2` (30), `STEER_DELTA` (15), `STEER_INVERT` (0),
  `PWM_AT_SPEED_MAX` (85).

## Build
1. Copy `comms/wifi_secrets.example.h` to `comms/wifi_secrets.h` and fill in SSID, password and the
   laptop's IP (gitignored — never commit it).
2. `powershell -ExecutionPolicy Bypass -File .\build_uf2.ps1` → `build/integrated_car.uf2`.
   Needs `track_purist/` next to this folder.

## Laptop side (`tools/`)
`mosquitto -c tools/mosquitto.conf -v`, allow TCP 1883 in the firewall, then
`pip install "paho-mqtt==2.1.0"` and `python tools/telemetry_monitor.py` (needs `tools/packets.py` and
`tools/barcode_map.py` next to it). Commands go as 1 byte each; the monitor decodes every reply.
The broker allows anonymous connections: use only on your own hotspot.
