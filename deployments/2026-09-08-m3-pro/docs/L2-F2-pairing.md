# L2 → F2 persistent ESP-NOW pairing

Both arms run verified firmware 0.84-s1 with verified original 4 MiB backups in their individually labeled backup directories.

| Arm | MAC | Role |
|---|---|---|
| L2 | fc:e8:c0:f8:c8:d8 | Unicast leader targeting only F2 |
| F2 | fc:e8:c0:f8:cc:24 | Follower accepting only L2 |

Both save AP boot mode. Each boot.mission was read back and executed after a software reboot. L2 rejects incoming control, registers only F2, releases torque, and enables unicast following. F2 restricts its sender to L2, enables torque, and enters follower mode. Exact saved commands are in each backup directory's boot-configured.json. L1/F1 settings were not changed.

Unique non-motion radio messages reached F2 after rebooting each arm separately. With L2 servo supply approximately zero volts, its feedback was invalid, positions were null, and motion transmissions remained zero. F2 received no packets during a separate ten-second observation. See L2-F2-safety-verification.json and L2-F2-radio-captures.json.

The user confirmed successful powered tracking on 2026-09-08; setup is complete. For the first powered test, support both arms and clear their movement areas; power L2 servo supply first, then F2, and test a small movement. The stock startup pose and the positioning step before leader torque release remain in the firmware. Physical unplug/replug cycles were not separately tested.
