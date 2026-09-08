# RoArm operating notes

User-confirmed cause of F1's unexpected 180-degree movement: F1's servo supply was powered while L1's servos were unpowered. Factory 0.84 still streamed default/stale joint data; default encoder position zero maps to +180 degrees for base and roll.

Use this power-up order for normal operation: support both arms, power the leader's servo supply, confirm valid leader feedback, then power the follower's servo supply. USB alone powers the controller, not the servo feedback electronics. The user's physical labels are L1 and F1; L2 and F2 are also identified and configured as a separate pair.

Firmware 0.84-s1 adds a fail-closed sender guard so an unpowered leader cannot stream motion targets even if the power-up order is reversed. This guard does not remove the stock startup pose sequence or independently validate mechanical assembly/calibration. On 2026-09-08 the user confirmed powered L1/F1 following works ("looks like it works"). This is a user-observed tracking check, not exhaustive mechanical or startup validation.

## Second pair

L2 (fc:e8:c0:f8:c8:d8) and F2 (fc:e8:c0:f8:cc:24) are backed up, flashed with 0.84-s1, and configured for persistent MAC-specific pairing. Individual reboot/radio checks and zero-voltage blocking passed. The user confirmed successful powered tracking on 2026-09-08. Firmware, backup, and pairing work is complete on all four arms. F2 joint rubber bands and packing are being handled by the user. See L2-F2-safety-verification.json. All four hardware identities are saved in arms.json.
