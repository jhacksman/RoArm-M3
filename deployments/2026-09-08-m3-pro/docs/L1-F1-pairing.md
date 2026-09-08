# L1 → F1 persistent ESP-NOW pairing

Both arms now run the verified local 0.84-s1 patch. Automatic pairing was restored after the invalid-feedback bug was fixed. L1 will not stream motion until its servo supply and all joint feedback are valid and fresh. See `FIRMWARE-SAFETY-FIX.md`. Pairing is saved in each controller's LittleFS `boot.mission`, which the firmware executes after Wi-Fi and ESP-NOW initialization on every boot. No computer is needed to reapply the configuration.

| Arm | Role | MAC | USB serial |
|---|---|---|---|
| L1 | Streaming unicast leader | fc:e8:c0:f8:bd:4c | 761c6985cc89ef11b3aab8a3ef8776e9 |
| F1 | Follower restricted to L1 | 2c:bc:bb:4d:c1:6c | 82809b8e3f7fef119e2c241cedd322a4 |

L1 addresses F1 directly. F1's sender whitelist contains only L1. L1 is configured to reject incoming control. Both save AP boot mode in `wifiConfig.json`, keeping the default radio setup aligned without joining a router. Changing Wi-Fi settings later can change the radio channel and disrupt ESP-NOW.

## Saved startup commands

L1:

```json
[
  {
    "T": 301,
    "mode": 0
  },
  {
    "T": 300,
    "mode": 0,
    "mac": "00:00:00:00:00:00"
  },
  {
    "T": 303,
    "mac": "2C:BC:BB:4D:C1:6C"
  },
  {
    "T": 210,
    "cmd": 0
  },
  {
    "T": 301,
    "mode": 2
  }
]
```

F1:

```json
[
  {
    "T": 301,
    "mode": 0
  },
  {
    "T": 300,
    "mode": 0,
    "mac": "FC:E8:C0:F8:BD:4C"
  },
  {
    "T": 210,
    "cmd": 1
  },
  {
    "T": 301,
    "mode": 3
  }
]
```

These arrays are exported for documentation; the actual boot.mission stores one JSON command per line following the mission header. Original empty missions and configured exports are saved in each arm's backup directory. Existing Wi-Fi config files were absent before setup.

## Verification

Both arms' startup files were read back. Each was rebooted separately, and logs confirmed restoration of its saved role. L1's registered peer and F1's sender restriction were restored. Unique non-motion text payloads sent over ESP-NOW from L1 appeared on F1 after the leader reboot and after the follower reboot. `L1-F1-verification.json` is the historical pre-fix radio test; its successful motion-frame delivery with unpowered servos exposed the safety bug. The authoritative post-fix result is `L1-F1-safety-verification.json`: zero motion packets at zero supply, with a separate text probe proving the link remains functional.

Tests used USB power only, as confirmed by the user. The user subsequently confirmed powered tracking works; latency was not measured. Software reboots were tested; physical unplug/replug power cycles were not separately tested. Configuration is stored in nonvolatile flash and is intended to apply on every normal startup, in either power-up order.

## Powered operation

The stock firmware attempts initial movement before running boot.mission. Its torque-release command also issues positioning commands before disabling leader torque. Support L1 and clear F1's movement area before applying servo power. L1 then releases torque for manual guiding; F1 enables torque and follows L1. The latest zero-voltage warnings reflect absent external servo power during these tests.

## Recovery and later arms

Full original 4 MiB backups, SHA-256 hashes, firmware manifests, and verification logs are stored separately in `backups/L1-fce8c0f8bd4c/` and `backups/F1-2cbcbb4dc16c/`. The original backups precede pairing configuration. Keep them private because full flash can contain credentials and settings.

L2 and F2 now have separate MAC-specific startup configurations; see L2-F2-pairing.md. Do not add broadcast peers to L1.
