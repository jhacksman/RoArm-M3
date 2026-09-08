# RoArm hardware work

- Read `arms.json` and `docs/OPERATING_NOTES.md` before controlling or flashing an arm. Identify hardware by USB serial and MAC, not only its current port name.
- User-confirmed failure: stock 0.84 streamed zero/stale encoder readings while L1's servos were unpowered, driving F1 toward incorrect targets, including a 180-degree base/roll offset. Radio delivery success alone is not a safe motion test.
- Preserve the 0.84-s1 supply/feedback/freshness guards. Never substitute fake valid readings or bypass a guard on a live arm to test transmission.
- Normal operating sequence: support the arms, power the leader's servos and verify valid feedback, then power the follower's servos. USB-only operation cannot validate physical joint tracking.
- The stock startup pose is separate from ESP-NOW pairing and remains in the firmware. Torque-release command 210 also issues positioning commands before releasing torque. Keep servo power off during unreviewed firmware work; do not infer that a passing radio test validates those movements.
- Keep the downloaded vendor source and original per-arm full-flash backups unchanged. Modify `src/RoArm-M3_safe/`, build with `tools/build-safe-firmware.sh`, and record exact firmware hashes and verification results.
