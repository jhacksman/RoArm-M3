# L1 — leader 1

- Hardware MAC: `fc:e8:c0:f8:bd:4c`
- USB bridge serial: `761c6985cc89ef11b3aab8a3ef8776e9`
- Chip: ESP32-D0WD-V3, revision 3.1
- Flash: 4,194,304 bytes (4 MiB)
- Serial port during preparation: `/dev/cu.usbserial-1110`
- Assigned on: 2026-09-07
- Intended firmware: factory RoArm-M3 version 0.84

`original-4MB.bin` is the pre-upgrade full-flash backup. Its completion and verification will be recorded here before firmware writing. It may contain Wi-Fi credentials and other device settings; keep it private.

L1 is an inventory designation. ESP-NOW leader configuration and pairing are separate and have not been applied. L2, F1, and F2 remain unassigned.

The first backup attempt at 460800 baud failed with a serial packet error. A subsequent connection without reset failed; the automatic download circuit successfully reconnected at 115200 baud.

## Completed upgrade

The full 4 MiB backup was saved and verified against the device flash (digest matched). `SHA256SUMS` records its file hash. The original and new partition tables are identical.

All four factory 0.84 images were written at the offsets in the project README and independently verified, with four matching digests. See `flash.log`, `firmware-verification.log`, and `firmware-manifest.json`. No full-chip erase was used; NVS and filesystem ranges were outside the write/erase ranges.

The arm was left in download mode. Startup behavior/version and ESP-NOW pairing have not been tested or configured. A subsequent EN/reset or power cycle can start arm motion.

Recovery: identify this same MAC first, then write `original-4MB.bin` at offset `0x0` with esptool using `--flash_mode keep --flash_freq keep --flash_size keep`, and verify the full image. This restores all captured flash contents, including settings, and should only be done when intentionally rolling this arm back.

The final verification logged a crystal-frequency estimate of 41.01 MHz normalized to 40 MHz; earlier identifications reported 40 MHz without that warning. All image digests matched.

## Subsequent startup diagnosis

After the user reset L1, serial capture showed normal firmware startup, successful LittleFS mount and ESP-NOW initialization, followed by servo feedback failures and a low operating voltage warning. Initial movement was skipped. The changing OLED `V:` field is supply voltage, not the firmware version. Check external servo power and the board power switch before further motion testing. The arm is now running firmware, not in download mode.

## Persistent pairing configuration

Saved startup commands are in `boot-configured.json`; the original empty startup mission is in `boot-original.mission`. AP mode is saved for every boot. L1 streams to F1 (`2c:bc:bb:4d:c1:6c`); F1 accepts only L1 (`fc:e8:c0:f8:bd:4c`). Reboot and radio verification are underway.

## Completed verification

The full original 4 MiB backup passed device digest verification. Factory 0.84 firmware was written and all four files independently verified. The saved startup mission was read back and executed after reboot. Both leader-first and follower-first availability were exercised by rebooting each arm separately; unique text payloads arrived at F1 after each test. Final 10-second sample: 16 successful L1 deliveries, 16 received F1 packets, zero delivery failures. See `pairing-verification.json` and `pairing-session.log`.

Both arms were tested on USB power only. Low servo voltage/feedback warnings are expected in this setup; physical joint tracking remains untested. The arms are now running their configured roles.

## 0.84-s1 safety fix deployed

Factory 0.84 was found to stream invalid leader positions with the leader servo supply off. Automatic following was paused during investigation, then restored after installing and verifying 0.84-s1 on both arms. `boot-configured.json` is the restored active configuration; `boot-paired-before-safety-pause.json` preserves the earlier pairing.

The final build passed flash verification and reboot testing. At approximately zero servo voltage L1 reports invalid feedback, null coordinates, increasing blocked cycles, and zero motion transmissions. F1 received no packets in a 10-second observation with its follower role active; a separate non-motion text probe arrived afterward, demonstrating that the radio link remained functional. Physical tracking has not been tested under servo power. See the final firmware manifest, verify-0.84-s1-final.log and L1-F1-safety-verification.json in the project docs.
