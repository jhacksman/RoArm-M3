# Hardware, calibration and control contract

Read the [deployment rules](../deployments/2026-09-08-m3-pro/AGENTS.md), [inventory](../deployments/2026-09-08-m3-pro/arms.json) and [operating notes](../deployments/2026-09-08-m3-pro/docs/OPERATING_NOTES.md) before any future hardware work. Keep the existing feedback/supply/freshness guards and original backups. Nothing here is a command execution recipe.

## Resolve before model or adapter approval

| Input | Required evidence / unresolved choice |
| --- | --- |
| Two target arms | Confirm F1/F2 versus another pair; labels, mechanical revision, firmware hash and actual servo/end-effector variants. Existing docs call them M3 Pro; current vendor servo naming differs from older root README text. Do not infer torque, mass or limits from that name alone. |
| Cell layout | Measured base separation and orientations, mounting plate/fasteners, table and fixtures, reachable safe zones, cable sweep, gripper/tool collision geometry and payload. No invented spacing is supplied. |
| Joint calibration | Vendor joint ↔ firmware field mapping, radians/degrees/ticks, signs, zero offsets, limits, wrap handling, shoulder dual-servo coupling and gripper angle ↔ opening. Distinguish five pose joints from the gripper: not a general six-axis pose arm. |
| Dynamics | Measured/verified link masses and inertias, payload center of mass, conservative velocity/acceleration/effort limits, backlash/compliance and stopping behavior. The vendor Xacro's zero effort/velocity values are not usable limits. |
| Coordinates/sensors | World-to-base transforms, TCP/tool offset, camera intrinsics and extrinsics, timestamps/clock origin, calibration revision and uncertainty. Recalibration invalidates queued plans. |
| Transport | Two USB data ports and cables, persistent identity even if USB bridges lack unique serials, independent power, newline framing, firmware baud, feedback schema/rate/age, acknowledgments, queue capacity and disconnect behavior. Use controller USB, not its LiDAR connector. Never assume host USB powers the servos or the arm regulator can power Thor. |
| Ownership | Explicit takeover/release protocol across USB, ESP-NOW, HTTP/web and saved missions. Preserve pairings; reject concurrent writers and stale reconnect queues. Boot/reset can move an arm. |
| Physical fault behavior | Accessible independent stop/power isolation, load support on torque loss, restart interlock and a supervised test plan. A ROS cancellation or JSON stop is not a demonstrated physical emergency-stop system. |

The [official Waveshare wiki](https://www.waveshare.com/wiki/RoArm-M3) documents serial at 115200 baud and alternative USB/UART wiring, not simultaneous use of both. Reconcile this with the exact deployed firmware before an adapter is enabled. Two connections do not imply synchronized execution.

## Command audit (source inspection only)

Evidence is pinned to repository main `0e801e0643d945f0674400414bb37a8cb6fd11d2`, in the deployed guarded source. Definitions are in [json_cmd.h](../deployments/2026-09-08-m3-pro/src/RoArm-M3_safe/json_cmd.h), dispatch and serial parsing in [uart_ctrl.h](../deployments/2026-09-08-m3-pro/src/RoArm-M3_safe/uart_ctrl.h), and motion routines in [RoArm-M3_module.h](../deployments/2026-09-08-m3-pro/src/RoArm-M3_safe/RoArm-M3_module.h).

- `T:104` is `CMD_XYZT_GOAL_CTRL` and dispatches to `RoArmM3_allPosAbsBesselCtrl` with Cartesian/attitude/gripper targets. **It is motion, not emergency stop.** Do not copy the previously reported shop-lifter mapping.
- `T:0` sets `StopFlag`; `T:999` clears it. The serial path detects compact strings by substring and clears its command queue on the stop match. The Cartesian interpolation loop checks the flag. This does not demonstrate coverage of direct joint commands, ESP-NOW motion, all formatting, or bounded stop latency.
- `T:210` calls `servoTorqueCtrl`; its release branch calls `Move_to_location()` before disabling torque. It must not be used as a harmless stop or passive release assumption.
- `emergencyStopProcessing()` is a separately defined torque-off/10-second-delay/torque-on helper. This audit did not establish a caller; **do not attribute its behavior to `T:0`**.

Before any controller exists, add offline command-conformance tests against a selected firmware version and audit all write paths, stop coverage, malformed input handling and queue flushing. Then agree on a physical stop architecture and supervised commissioning sequence. No packet was sent to a robot during this work.
