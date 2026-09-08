# Repository snapshot — 2026-09-08

Saved in jhacksman/RoArm-M3 under deployments/2026-09-08-m3-pro. Existing repository content and history are preserved.

Includes the patched source and tests, exact deployed firmware and manifests, original factory firmware, all four full original flash backups and logs, per-arm identities and boot configurations, verification reports, operating notes, downloaded vendor archives, and extracted vendor source/libraries.

Installed Python environments, Arduino CLI executables, downloaded compiler caches, and extracted compiler packages are machine dependencies and are not versioned. The vendor's extracted build directory duplicates the retained original source ZIP; recover its ELF/map/prebuilt files by extracting that ZIP. SHA256SUMS.snapshot records every included file. No backup or firmware binary was modified for publication.

Credential review found factory RoArm-M3 / 12345678 AP settings and no configured station credentials in the recorded configuration. The original NVS AP password and SSID match those defaults on all four devices. Text files were scanned for token/private-key patterns with no matches. Future flash backups need a fresh credential review before public upload.

## Recreate machine dependencies

Use a working Python 3.12+ to create tools/flash-env-arm64, then install tools/flash-requirements.txt with that environment's pip. Place the macOS ARM64 Arduino CLI at tools/arduino-cli. The working Arduino data and libraries used here live under ~/.cache/roarm-m3-build, configurable through ROARM_BUILD_CACHE.

Install esp32:esp32 core 3.0.7 using the Espressif board-manager URL recorded in tools/arduino-cli.yaml. In the build cache's arduino-cli.yaml, configure data as <cache>/arduino-data and user as <cache>/arduino-user. Copy the retained vendor libraries to <cache>/arduino-user/libraries, replacing ArduinoJson with 7.3.1. Install ESP Async WebServer 3.6.0 and Async TCP 3.3.2. Do not copy the vendor/RoArm-M3_example sketch as a library.

Run tools/build-safe-firmware.sh. The script checks the guard's compile-time tests, builds ordinary C++ with the empty .ino adapter, checks the partition table and application size, and updates the firmware manifest. Its native ARM64 esptool wrapper and ctags bypass were necessary for the Intel-only helpers bundled with this ESP32 core. Exact pinned build settings are in firmware/0.84-s1/manifest.json.

Prebuilt firmware flashing needs only the Python/esptool environment. Always identify an arm by USB serial and MAC and preserve its original backup before writing. See AGENTS.md and docs/OPERATING_NOTES.md for the operating sequence.
