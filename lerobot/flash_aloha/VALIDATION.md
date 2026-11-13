# Validation record and acceptance plan

Preparation date: November 12, 2026. Execution host: Apple Silicon macOS development machine. The eventual target is a different computer. The observed Python version was 3.14.2; the preparation tools use only standard-library features compatible with Python 3.10+. Compatibility with every Python/OS combination has not been tested.

## Completed offline validation

Run from the RoArm repository root:

```sh
python3 lerobot/flash_aloha/run_offline_checks.py
python3 lerobot/flash_aloha/inspect_setup.py --feedback \
  lerobot/flash_aloha/fixtures/position.synthetic.json
```

The suite currently contains 16 tests. They cover finite/missing/boolean/nonfinite feedback fields, guard rejection, native joint ordering, immutable decoded state, absent velocity, numeric T=102 command formatting, rejection of seven/twelve-value single-arm targets, explicit limits, bounded steps, nonzero speed/acceleration and detection of changed/missing reference files. Test poses, voltages and limits are synthetic and are never sent to hardware.

Historical fixture rejection:

```sh
python3 lerobot/flash_aloha/inspect_setup.py --feedback \
  deployments/2026-09-08-m3-pro/firmware/0.84-s1/leader-zero-voltage-feedback.json
```

Expected result: exit status 2, invalid guard flags, null joint state and no claim of deployment readiness. This is a checked-in September fixture, not a newly collected measurement. Host-only inspection has no readiness success mode: even a structurally valid saved packet reports missing velocity and unverified live timing.

Pinned reference verification, with the actual local Flash ALOHA checkout path substituted:

```sh
python3 lerobot/flash_aloha/verify_sources.py \
  --flash-root /absolute/path/to/flash-aloha --roarm-root .
```

These two supplied roots cover 26 reference files; all matched during preparation. The manifest also records a checksum for the pinned official SDK API document. To verify that additional local checkout, pass `--sdk-root /absolute/path/to/waveshare_roarm_sdk`. An omitted checkout is not counted as verified. The manifest is scoped evidence, not a checksum of entire upstream repositories.

## Checks for the future implementation

After adding a real backend/integration, run upstream tests appropriate to those changes. The current upstream `AGENTS.md` names `scripts/tests/test_standard_capture.py` and image-processing parity checks. Also cover the robot/configuration/runtime tests and new RoArm lifecycle/transport tests. Resolve dependencies on the chosen development host; the full upstream runtime and test suite were not installed or run for this package. No training run or model benchmark was performed.

Minimum simulation cases for the eventual adapter:

| Case | Required outcome |
| --- | --- |
| Import/build/config validation | No connection or motion side effects; reject missing identity/role/limits |
| Legacy position-only packet | Refuse live capture as a complete position/velocity observation |
| Malformed, truncated, oversized or interleaved debug lines | Bounded parsing; no stale state accepted as a new reply |
| Stale sequence, reset, invalid servo, zero supply, excessive sample span | Fault according to the reviewed state/stop contract |
| Six/twelve dimensions and role ordering | Exact shapes in data, normalization statistics, checkpoint and runtime |
| One arm fails during two-arm connection or target submission | All owned resources follow the defined partial-failure cleanup path |
| Invalid target, gripper unit mismatch, oversized step or expired command | Reject before target delivery; never silently clip or pad |
| ESP-NOW/host handoff | Single writer, positive acknowledgement and recoverable paired configuration |
| Capture and preprocessing | Exact frame IDs/clock provenance retained and train/inference image parity |

Passing these simulations does not substitute for the measurements below.

## Pending target and hardware acceptance

Every item is **pending**. The preparation scope requires no answers or physical work now. Record dates, device identities, firmware/checkpoint hashes, operator observations and retained logs when a later agent performs these checks.

| Acceptance item | Evidence needed |
| --- | --- |
| Target installation | OS/architecture, supported Python, training/inference compute, dependency install and offline-suite results |
| Hardware identity | USB serial and MAC matched to inventory; explicit physical left/right assignment |
| Serial connection behavior | Supported-arm test of opening/closing, DTR/RTS, reset/startup motion and exclusive ownership |
| Mechanical calibration | Measured directions, encoder offsets, gripper angle/opening relationship and joint/step bounds |
| Complete state telemetry | Position/velocity from the same poll, signed rad/s conversion, invalid/reset/rollover cases and bounded sample age |
| Supply/feedback guards | Preserved behavior on powered/unpowered leaders and communication failures; no fabricated valid feedback |
| Control/stop behavior | Operator-tested normal stop, command expiry, host death, transport loss and ESP-NOW ownership transfer |
| Sustained timing | Serial round-trip distribution, MCU sample span, follower tracking lag, camera frame age/skew and missed deadlines |
| Dataset integrity | Exact recorded action semantics, role/dimension metadata, frame references and playback/export audit |
| First policy rollout | Conservative supervised soft-object task, reproducibility and retained outcome logs |

Follow the deployment operating notes before powering or controlling an arm. Do not reuse ARX homing/gravity-tuning commands, a synthetic target, SDK gripper example bounds or the example 100 Hz rate as a RoArm operating setting. Keep all physical observations distinct from the September historical setup and from these offline tests.
