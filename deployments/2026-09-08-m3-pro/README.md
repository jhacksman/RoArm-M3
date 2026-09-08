# RoArm-M3 Pro firmware upgrade

**Setup complete for L1, F1, L2, and F2: firmware 0.84-s1.** All four have verified original backups and verified firmware. Separate L1 → F1 and L2 → F2 pairings restore automatically on boot. Radio and zero-voltage blocking checks passed, and the user confirmed powered tracking works on both pairs. F2 joint rubber bands and packing are user-owned physical tasks.

See [inventory](arms.json), [L1/F1 pairing](docs/L1-F1-pairing.md), [L2/F2 pairing](docs/L2-F2-pairing.md), [safety fix](docs/FIRMWARE-SAFETY-FIX.md), and [operating notes](docs/OPERATING_NOTES.md).

The setup and machine-audit notes below are historical. Factory 0.84 artifacts are retained for recovery and provenance; the deployed guarded application and manifest are in `firmware/0.84-s1/`. The working build command is `tools/build-safe-firmware.sh`.

Downloaded 2026-09-07 from the current links on https://www.waveshare.com/wiki/RoArm-M3 .

The official source package `RoArm-M3_example_20260701.zip` identifies itself as version **0.84**. Both it and the factory package contain application binaries dated July 6, 2026. ESP-NOW leader/follower support is present. This does not establish a latency improvement over the firmware currently on your arms, whose version has not yet been read.

## Ready for upgrade

- `downloads/RoArm-M3_FACTORY-260701.zip`: original factory package.
- `firmware/official-0.84/`: extracted current factory binaries. Ignore the factory ZIP's old `combine/target.bin` and `dl_temp` files.
- `vendor/RoArm-M3_example/`: unchanged official source and prebuilt firmware.
- `downloads/Libraries.zip`: official development libraries.
- `docs/`: saved vendor setup and ESP-NOW documentation.
- `tools/flash-env-arm64/`: local Python environment for esptool 4.8.1. Run it through `./tools/esptool`.
- `tools/flash-env/`: older environment with a broken Homebrew Python reference; do not use it on this Mac.

Connect one arm at a time using the controller's USB data/programming port (interface 9 in the Waveshare diagram) and a data-capable cable. A USB serial device appeared at `/dev/cu.usbserial-1110` during the September 7 machine audit; its board identity is not yet confirmed and the port may change. Before reset or flashing, support the arm and clear its motion area: the official firmware moves to its initial pose at startup.

Next steps: identify the board and flash size, back up its full flash, flash the four current factory binaries at the offsets below, then confirm version 0.84 on startup. Repeat for the other arm before pairing them. Do not use a full-chip erase or merged 4 MB image as the first step: preserve existing settings and filesystem where possible.

Offsets from the factory `multi_download.conf`:

| Offset | Binary |
|---|---|
| 0x1000 | RoArm-M3_example.ino.bootloader.bin |
| 0x8000 | RoArm-M3_example.ino.partitions.bin |
| 0xe000 | boot_app0.bin |
| 0x10000 | RoArm-M3_example.ino.bin |

Use esptool's `--chip esp32 --port PORT flash_id` for identification and `read_flash 0 ALL backup.bin` for a backup before writing. Match the detected board before proceeding. Flash with `write_flash --flash_mode keep --flash_freq keep --flash_size keep` and the four offset/file pairs above. Verify against the same files afterward.

## Machine audit — 2026-09-07

This Mac is Apple Silicon, running macOS 27.0. The previous environment referenced an absent `/opt/homebrew/opt/python@3.14/bin/python3.14`; the replacement uses the Codex bundled Python 3.12 runtime. If that runtime is removed, recreate the environment with a working Python and `tools/flash-requirements.txt`.

The three downloaded archives match the saved SHA-256 manifest, and all four extracted firmware binaries match the factory archive byte for byte. This verifies local consistency, not an independently authenticated vendor signature.

The local Arduino CLI launches, but reports no installed board platforms: building modified source is not yet configured. Prebuilt firmware flashing does not require the Arduino compiler. The cached esptool 4.6 macOS executable is incompatible with this machine without an Intel compatibility runtime; use `./tools/esptool` instead.

Offline tool check: `./tools/esptool version`. List ports without resetting a board: `./tools/flash-env-arm64/bin/python -m serial.tools.list_ports -v`.

The connected USB bridge identifies as Silicon Labs CP2102N (VID:PID `10c4:ea60`), and macOS exposes its serial port without an additional driver installation. The initial machine audit preceded flashing; see the completed L1/F1 inventory and pairing record below for the subsequent upgrades.

Use the Type-C connector labeled **USB**, interface **9**, for serial and programming. The **LADAR** connector, interface **8**, is for LiDAR. Normal serial communication needs no button sequence. The board normally enters download mode automatically for flashing. If esptool stalls at `Connecting...`, hold **BOOT**, press **EN/reset** for about one second, release **EN**, then release **BOOT**. This is the fallback described in the [vendor FAQ](https://www.waveshare.com/wiki/RoArm-M3#FAQ); the power switch is not part of that button sequence.

## Leader/follower setup after upgrading

Inventory is tracked in `arms.json`. L1 (`fc:e8:c0:f8:bd:4c`) and F1 (`2c:bc:bb:4d:c1:6c`) run 0.84-s1 with persistent pairing and verified zero-voltage motion suppression. Both have original verified 4 MiB backups. L2 (`fc:e8:c0:f8:c8:d8`) and F2 (`fc:e8:c0:f8:cc:24`) are also fully configured and user-tested as a separate pair. See [pairing details](docs/L1-F1-pairing.md).

Obtain both MAC addresses with `{"T":302}`. Pair explicitly to the other arm rather than enabling broadcast control. Configure only after the follower has room to move and the leader is supported before releasing torque. Command details are saved in `docs/ESP-NOW-Control.html`; use the actual MAC addresses when configuring.

## Remaining latency investigation

The current official source still prints on each ESP-NOW transmission and in its send callback; samples six servos sequentially before transmitting; and performs servo control in the receive callback. These are candidates for measurement and optimization, not proof of the cause of your observed lag. First upgrade both arms and measure their behavior.
