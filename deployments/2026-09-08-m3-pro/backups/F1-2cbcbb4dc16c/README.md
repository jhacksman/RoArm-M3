# F1 — follower 1

MAC: `2c:bc:bb:4d:c1:6c`

USB serial: `82809b8e3f7fef119e2c241cedd322a4`

ESP32-D0WD-V3 revision 3.1, 4 MiB flash. Intended leader: L1 (`fc:e8:c0:f8:bd:4c`).

Backup and upgrade in progress. No pairing configuration has been applied yet.

## Persistent pairing configuration

Saved startup commands are in `boot-configured.json`; the original empty startup mission is in `boot-original.mission`. AP mode is saved for every boot. L1 streams to F1 (`2c:bc:bb:4d:c1:6c`); F1 accepts only L1 (`fc:e8:c0:f8:bd:4c`). Reboot and radio verification are underway.

## Completed verification

The full original 4 MiB backup passed device digest verification. Factory 0.84 firmware was written and all four files independently verified. The saved startup mission was read back and executed after reboot. Both leader-first and follower-first availability were exercised by rebooting each arm separately; unique text payloads arrived at F1 after each test. Final 10-second sample: 16 successful L1 deliveries, 16 received F1 packets, zero delivery failures. See `pairing-verification.json` and `pairing-session.log`.

Both arms were tested on USB power only. Low servo voltage/feedback warnings are expected in this setup; physical joint tracking remains untested. The arms are now running their configured roles.

## 0.84-s1 safety fix deployed

Automatic following was paused after invalid leader data caused unexpected follower movement. Both arms now run verified 0.84-s1. Pairing is restored at boot, with a sender guard that blocks motion packets unless supply and complete fresh joint feedback are valid. Final zero-voltage and radio-probe evidence is in `docs/L1-F1-safety-verification.json` at the project root. Powered tracking remains untested. `boot-configured.json` is the restored active startup configuration.
