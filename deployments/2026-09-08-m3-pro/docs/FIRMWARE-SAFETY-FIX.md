# 0.84-s1: reject invalid leader feedback

## Root cause

In factory 0.84, `getFeedback()` leaves the previous encoder position in memory when a servo read fails. The global feedback array starts at zero. The loop nevertheless converts those entries to angles and streams them over ESP-NOW. Zero maps to +180 degrees at base and roll. With L1's servo supply off and F1 powered, that can send F1 toward extreme positions. Successful radio delivery did not establish valid motion data.

## Change

The patched source is in `src/RoArm-M3_safe/`; the vendor source is preserved. All automatic single/group streams and explicit ESP-NOW pose-send functions now require a complete sample and a fresh supply check before calling `esp_now_send` for motion.

- Read the INA219 supply register with checked I2C transactions and explicit byte order. Missing sensor, incomplete reads, overflow, or a supply outside the firmware's existing protection limits (strictly above 6.0 V and below 12.9 V) blocks streaming.
- Require successful reads and encoder values within 0–4095 for all seven servos, including the shoulder's paired actuator.
- Check supply before and after collecting feedback; reject samples taking over 100 ms. Recheck supply immediately before transmission and reject samples older than 100 ms, including across millis rollover.
- Invalidate the sample before collecting a replacement. Failed/partial reads do not update the joint-angle snapshot.
- Position feedback command 105 adds `firmware`, `feedback_valid`, `motion_allowed`, `motion_supply_v`, `motion_tx_frames`, and `motion_blocked_cycles`. Invalid position/angle fields are returned as null.
- Non-motion text probes remain possible with servo power off. A blocked guard does not erase peers or change the configured role; valid supply and complete fresh readings are required before motion streaming resumes.

## Build and scope

Run `tools/build-safe-firmware.sh`. The local toolchain is under `~/.cache/roarm-m3-build`: ESP32 Arduino 3.0.7, ArduinoJson 7.3.1, ESP Async WebServer 3.6.0, Async TCP 3.3.2, plus the other vendor libraries. The original bundled library archive contains ArduinoJson 6, which does not support this sketch's default-constructed JsonDocument; version 7 is pinned for this source.

The sketch implementation is ordinary C++ in main.cpp with an empty .ino adapter. The Intel-only ctags prototype generator is bypassed because no generated prototypes are needed; all code is compiled normally by the ARM64-hosted ESP32 compiler.

Only the application at 0x10000 is replaced. The new application fits the existing 0x140000-byte slot, and its generated partition table matches factory 0.84 byte for byte. Bootloader, partition table, NVS and filesystem are retained. Exact source/application hashes are in `firmware/0.84-s1/manifest.json`.

## Verification and limits

Compile-time C++ tests cover normal supply, zero/noisy/invalid/overvoltage values, missing joint feedback, invalid encoder range, slow samples, stale samples, and timestamp rollover. Hardware checks exercise USB-only leader mode and confirm zero motion packets while F1 is actively listening. A non-motion text probe confirms the radio still works, so a broken link cannot account for the zero-packet result. Reboot verification and final-build evidence are saved with the per-arm backups.

These tests do not verify powered joint tracking or measure live healthy-snapshot timing. The stock startup pose, torque-release positioning sequence, and mechanical calibration remain separate from this fix. This patch is not a receiver watchdog or an emergency-stop system. Use the operating sequence in `OPERATING_NOTES.md` for the first powered check.
