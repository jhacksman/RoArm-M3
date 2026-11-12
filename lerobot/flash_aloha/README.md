# Flash ALOHA on RoArm-M3 Pro

Source verified November 12, 2026:

- [Original project and paper links](https://johnyao93.github.io/flash-aloha/)
- [Flash ALOHA code, inspected revision 4172a8e](https://github.com/JohnYao93/flash-aloha/tree/4172a8ed8cadb6eeb4a59285826e83cb00a2f01c)
- [This RoArm deployment, inspected revision 091f31d](https://github.com/jhacksman/RoArm-M3/tree/091f31d14f728b74f41ac3bbfff5a876b6a942e8/deployments/2026-09-08-m3-pro)

The learning approach is a plausible fit for the existing L1→F1 and L2→F2 setup. The released controller cannot operate these arms yet. This directory contains offline inspection tools and an integration specification; it contains no robot controller, collector or policy runner. No physical operation has been verified by this work.

## What the source actually requires

| Interface | Flash ALOHA at inspected revision | RoArm-M3 Pro at inspected revision | Required work |
| --- | --- | --- | --- |
| Hardware selection | Factory accepts only `arx`, models `L5` and `Trossen_L5` | USB serial JSON, guarded firmware `0.84-s1` | Add a separately named backend and validate configuration before connecting |
| Joint vectors | Seven values per ARX arm | Six values per RoArm: base, shoulder, elbow, wrist pitch, wrist roll, gripper | Six per arm or twelve for two follower arms; preserve explicit left/right roles |
| Gripper | ARX native opening width | Native angular position in radians | Declare units; never apply the ARX width conversion |
| Observation | Position and velocity from one state result per arm, then shared camera-cache snapshot | `T=1051` contains joint radians and guard flags; velocity is not serialized | Export a coherent state containing position, velocity, validity and sample timing; verify speed signs/units |
| Failure handling | Native damping, gravity compensation and watchdog | Position servos; equivalent ARX stop mode is not established | Specify and physically validate a RoArm stop/watchdog strategy before policy motion |
| Timing | Example control rate 100 Hz; ARX motor interpolation 500 Hz | Serial feedback/control throughput and latency unmeasured here | Measure sustained rates and delay, then configure compatible collection/training/deployment timing |
| Demonstrations | Direct hand guiding | Existing guarded leader/follower pairs | Capture actual follower state alongside leader command targets with explicit timing; validate the action/state relationship |

The firmware already reads `ReadSpeed(-1)` while reading each servo's position, but `RoArmM3_infoFeedback()` does not export those speeds. The SDK's documented `feedback_get()` returns position/posture and joint radians, not velocities. Do not fill missing velocities with zeros or silently derive them from unrelated position reads. MCU sampling is sequential across servos; expose its duration and validity rather than claiming simultaneous sensor acquisition.

Sources: [hardware factory](https://github.com/JohnYao93/flash-aloha/blob/4172a8ed8cadb6eeb4a59285826e83cb00a2f01c/src/hardware/arm_factory.py), [arm lifecycle](https://github.com/JohnYao93/flash-aloha/blob/4172a8ed8cadb6eeb4a59285826e83cb00a2f01c/src/hardware/arm_interface.py), [ARX backend](https://github.com/JohnYao93/flash-aloha/blob/4172a8ed8cadb6eeb4a59285826e83cb00a2f01c/src/hardware/backends/arx.py), [capture contract](https://github.com/JohnYao93/flash-aloha/blob/4172a8ed8cadb6eeb4a59285826e83cb00a2f01c/src/contracts/standard_capture.py), [RoArm telemetry](https://github.com/jhacksman/RoArm-M3/blob/091f31d14f728b74f41ac3bbfff5a876b6a942e8/deployments/2026-09-08-m3-pro/src/RoArm-M3_safe/RoArm-M3_module.h), [official M3 SDK API](https://github.com/waveshareteam/waveshare_roarm_sdk/blob/main/doc/roarm_m3_en.md).

## Run the offline check now

From this repository's root, using Python 3.10 or later:

```sh
python3 lerobot/flash_aloha/inspect_setup.py
python3 -m unittest discover -s lerobot/flash_aloha -p 'test_*.py' -v
```

The inspector lists candidate USB serial device paths without opening ports, changing DTR/RTS, starting cameras or importing a robot SDK. Port enumeration cannot identify an arm. Match each device's USB serial and MAC to the existing [inventory](../../deployments/2026-09-08-m3-pro/arms.json) before operation.

For an existing saved **single JSON feedback object**, run:

```sh
python3 lerobot/flash_aloha/inspect_setup.py --feedback /absolute/path/to/feedback.json
```

It checks the packet type, guarded firmware identifier, guard flags, finite positive reported supply and six finite joint radians. It preserves firmware ordering and never synthesizes velocity. `saved_position_packet_ok` describes only the saved packet; it does not verify calibrated limits, live freshness, correct hardware identity, camera alignment or physical readiness. `robot_deployment_ready` remains false. Exit status 0 means inspection completed, 2 means the saved input failed the packet check or could not be parsed.

The checked-in historical zero-voltage fixture is expected to fail with exit status 2:

```sh
python3 lerobot/flash_aloha/inspect_setup.py --feedback \
  deployments/2026-09-08-m3-pro/firmware/0.84-s1/leader-zero-voltage-feedback.json
```

## Bring-up sequence

1. Identify the computer that will connect to the arms and cameras, the available cameras and the training/inference computer. The first audit on Jack's Apple Silicon Mac found only Bluetooth/debug serial ports, with no RoArm USB serial device. This does not establish what is connected to another computer. Flash ALOHA's full runtime and Apple GPU support have not been tested here; do not assume the ARX installation commands work on this Mac.
2. Connect and identify one leader/follower pair. Review the existing [operating notes](../../deployments/2026-09-08-m3-pro/docs/OPERATING_NOTES.md). Support the arms; for normal operation power the leader's servo supply, verify valid feedback, then power the follower's servo supply. USB alone does not power the servo feedback electronics. Startup can move the arm, and command 210 also issues positioning commands before torque release. Preserve the deployed guards and backups.
3. Resolve the state and stop-mode contracts above. Add simulated lifecycle tests for stale/invalid feedback, timeout, malformed and out-of-limit targets, startup anchoring, and disconnects. Separately validate physical stop and serial reset behavior with the operator. Explicitly hand off control ownership between ESP-NOW following and policy execution so two writers cannot command the follower.
4. Add the backend/configuration and collection path while retaining `capture_observation`, exact `recording_fields()`, shared camera timing, standard recording storage and shared image preprocessing. Preserve all recorded native streams. Use the same joint units, roles, observation/action dimensions and image profile in training and inference. Do not repurpose the ARX hand-guiding commands or launch an ARX example config on RoArms.
5. Measure serial round-trip time, camera age/skew and follower lag. Select an initial control rate from those measurements; the example 100 Hz rate is not a RoArm benchmark. Validate one slow, operator-supervised pair before enabling both pairs.
6. Collect one simple task with a soft object, inspect the synchronized dataset, train ACT, and validate conservative execution. Evaluate action smoothing and increased speed only after repeatable behavior. The paper's demonstration count and success rates are results on its own platform, not a guarantee for RoArms.

The existing `lerobot/examples/record_teleoperation.py` and `deploy_imitation_model.py` are historical sketches: they send string-valued commands such as `GetJointPositions` and `AngleCtrl`, whereas the deployed firmware dispatches numeric `T` commands. They do not implement Flash ALOHA's camera/state capture contract. Their presence is not evidence of a working learning pipeline.

## Current implementation boundary

[Flash ALOHA's AGENTS.md](https://github.com/JohnYao93/flash-aloha/blob/4172a8ed8cadb6eeb4a59285826e83cb00a2f01c/AGENTS.md) requires position/velocity from one state result and says: “If a requested change needs one of these to behave differently, say so and stop at that boundary instead of making the change.” A RoArm stop policy would differ from the current ARX-specific damping behavior. This work stops before changing that behavior. The offline inspector neither changes those contracts nor implements an alternate live path.
