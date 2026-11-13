# Implementation map and unresolved contracts

All source observations below refer to the November 12, 2026 revisions in `sources.json`. File paths in the first table are relative to the Flash ALOHA repository. These are integration requirements and proposals, not claims that a port is implemented.

## Changes needed beyond a serial adapter

| Source path and symbol | Verified behavior | Work for the RoArm port |
| --- | --- | --- |
| `src/hardware/arm_factory.py`: `normalize_hardware_config`, `build_arms` | Only `arx`, `L5`/`Trossen_L5`, and CAN interfaces accepted | Add an explicitly named RoArm backend, serial identity/configuration and inert construction |
| `src/config/task_config.py`: `load_task_config`, `TaskConfig.legacy` | Rejects other robot types; legacy view indexes CAN fields | Extend schema and compatibility view without relabeling serial ports as CAN |
| `src/config/task_config.py`: `default_start_position_path` | Shared fallback is an ARX start pose | Require a RoArm-specific saved/calibrated pose and preserve arm roles |
| `src/hardware/arm_interface.py`: `ManagedArm` | Watchdog failures and close paths assume native damping | Implement a verified RoArm lifecycle; a position hold or torque release cannot be silently called equivalent damping |
| `src/hardware/runtime.py`: `HardwareRuntime.require_policy` | Derives dimensions from arms but validates rate using an all-zero target | Separate timing validation from pose validation if zero is outside calibrated RoArm limits; retain strict target checking |
| `src/hardware/runtime.py`: `read_state`, `capture`, `submit_target` | Shared capture, cross-arm feedback skew and validation of all targets before submission | Reuse after proving state timing and failure semantics; do not bypass its checks |
| `src/recording/record_episode_single.py`: `RunContext`, `normalize_robot_type`, `load_start_qpos`, `ARXBackend` | Seven/fourteen dimensions, ARX-only collection, ARX gripper and start-pose handling | Add dimension/role-driven RoArm collection and an explicit demonstration-action contract |
| `src/inference/inference_remote.py`: `configure_runtime_shape` | Sets action dimensions to seven/fourteen and computes arm count using seven | Derive dimensions and arm count from the selected backend/roles and checkpoint metadata |
| `src/inference/inference_direct.py`: `eval_bc`, start-pose/arm helpers | Treats action dimension greater than seven as dual-arm; multiple CUDA calls and ARX-specific helpers | Audit the chosen deployment path for six/twelve values and native gripper units |
| `src/inference/shared_hardware.py`: `resolve_start_action`, `run_shared_rollouts` | Reuses shared hardware, remote policy and direct start-pose helpers; comments assume gripper width in metres | Candidate integration path, but its remote runtime and inherited pose/gripper semantics still need the changes above |
| `src/policy/imitate_episodes_train.py`: `forward_pass`, `train_bc`, `cuda_prefetch` | Calls `.cuda()` and CUDA streams directly | Plan NVIDIA/CUDA training, or explicitly port/test device handling; a CPU or Apple GPU flag alone is insufficient |
| `src/policy/imitate_episodes_train.py`: single-arm metadata helpers | Resolves ARX side/CAN fields | Export accurate RoArm model/role metadata without inventing CAN devices |
| `src/contracts/standard_capture.py`, `standard_recording.py`, `standard_image_processing.py` | Shared state→camera ordering, recording references and preprocessing | Reuse these contracts; do not fork them to make a RoArm demo pass |

The RoArm code paths used for the other observations are under `deployments/2026-09-08-m3-pro/src/RoArm-M3_safe/`: `RoArm-M3_module.h`, `uart_ctrl.h`, `esp_now_ctrl.h` and `feedback_safety.h`. The official SDK API reference is also pinned in the manifest.

## Native joint mapping

| Index | Joint | Feedback key | T=102 target key | Encoder-position derivative sign |
| --- | --- | --- | --- | --- |
| 0 | Base | `b` | `base` | Negative |
| 1 | Shoulder, driving servo | `s` | `shoulder` | Positive |
| 2 | Elbow | `e` | `elbow` | Positive |
| 3 | Wrist pitch | `t` | `wrist` | Positive |
| 4 | Wrist roll | `r` | `roll` | Negative |
| 5 | Gripper angle | `g` | `hand` | Positive |

Signs follow `calculateRadByFeedback`; they are not proof of the raw speed register's units or sign representation. The shoulder has two physical servos but one logical joint. Keep both servos' validity checks. Each arm contributes six policy values; two followers contribute twelve, ordered by explicit roles, not USB discovery order.

The deployed `RoArmM3_handJointCtrlRad` clamps encoder targets to 700–3396, while the inspected SDK documentation lists a different angular gripper range. Preserve this discrepancy for calibration; do not choose operating limits from the SDK example or the synthetic tests. The pure formatter requires every bound and step limit to be provided explicitly for this reason. The gripper is an angle, with no ARX opening-width conversion.

## State telemetry proposal

Current `T=105` requests cause a servo poll and a `T=1051` response. Position, validity and supply information are exported; speed is read internally by `getFeedback` but is absent from the response. The host's receipt time does not reveal the sensor sample's age.

A later firmware change should add a versioned state payload without altering existing joint commands or guard decisions. The minimum useful logical fields are:

| Field | Intended meaning |
| --- | --- |
| Schema/version and firmware identity | Distinguish legacy position-only feedback from a coherent state contract |
| Boot/session identity and monotonically advancing sample sequence | Detect reset, replay and duplicate samples |
| MCU sample-start/end times and duration | Bound the sequential servo sampling span; handle MCU timer rollover explicitly |
| All relevant servo validity, supply and guard results | Preserve failure provenance and the existing dual-shoulder checks |
| Joint positions and raw signed servo speeds from that same poll | Avoid rereading position/velocity for one observation |
| Verified speed units and coordinate convention | Permit a documented six-value rad/s vector after register interpretation is confirmed |
| Accepted command sequence/targets when collecting demonstrations | Associate actions with the actual follower control path |

This is a proposal, not an assigned wire schema or an implementation. Review payload size and serialization overhead before selecting field names or a command ID. Invalid samples must remain invalid and must not publish previously valid numeric state as a fresh sample. Preserve the existing supply and freshness guards; telemetry serialization must not insert independent servo polls.

Confirm raw-speed conversion against the bundled servo library, vendor documentation and measurements. The derivative magnitude of the position mapping is `2π / ARM_SERVO_POS_RANGE` per encoder step, but applying it to speed requires evidence that the register represents steps per second. Record this evidence and test signs for every logical joint. Do not fill qvel with zero or change the capture contract to accept position-only legacy packets.

Map device clocks to host monotonic time with an explicit uncertainty bound. Independent MCU uptimes are not a shared clock. Cross-arm skew limits must account for transport delay and sample span, not merely Python receipt order. A schema extension cannot on its own prove synchronized sampling.

## Demonstration actions and control ownership

The existing pairs follow over ESP-NOW. Reading the leader and follower at nearby times does not reveal exactly which target the follower accepted. Select and document one collection approach before recording training data:

1. Preserve ESP-NOW following and instrument the accepted command's sequence, target and timing in follower telemetry. Retain the leader's paired identity. Use the follower's actual state as observation, with explicitly associated target actions.
2. Implement a host serial teleoperation bridge that exclusively owns follower targets during collection. Record the exact host-submitted targets. This changes the previously tested following path and needs its own latency, power-loss and control-transfer validation.

Neither bridge is implemented here. Do not allow ESP-NOW and host policy commands to interleave. Control-mode changes must be acknowledged, fail closed and have a defined return path to the normal paired configuration. Do not permanently overwrite the saved pairing just to run a policy.

All live capture must use `capture_observation`, qpos/qvel from one result per arm, the shared camera cache and exact `recording_fields()`. Record leader targets/command sequence as additional action metadata; they must not replace the captured follower state or cause a second camera selection. Use standard recording buffers, native retained camera streams, matching deterministic preprocessing and the prescribed capture garbage-collection lifecycle.

## Stop and watchdog contract

ARX damping includes gravity support and controller-native behavior. The following RoArm alternatives require separate assessment:

| Candidate | Unresolved issue |
| --- | --- |
| Position hold | A last position target may still be in transit; delayed or stale hold commands can move the arm |
| Torque release | Can drop the arm or load; command 210 also performs positioning before release in this firmware |
| T=0 emergency flag | Sets `StopFlag`; establish which paths observe it, including ESP-NOW callbacks and pending servo commands |
| MCU command-expiry watchdog | Needed to reason about host death or lost transport; its final servo action still needs a validated mechanical policy |

Do not equate a Python thread timeout or a successful serial write with a physical stop. Review process stalls, transport loss, stale feedback, half-connected bimanual startup, target rejection and operator cancellation. Validate the selected behavior on one supported arm before two-arm use. The existing upstream instructions protect capture and hardware behavior; surface any necessary contract change explicitly rather than hiding it behind an ARX method name.

## Implementation stages and completion evidence

1. **Reference and protocol preparation — provided here.** Reproduce the local tests and verify the pinned sources. Read the current deployment inventory/notes and record any later source drift.
2. **Telemetry and failure design.** Produce an additive, reviewable firmware patch plus simulated tests for coherent state, invalid samples, sequence/reset/wrap handling and guards. Prove the stop/ownership strategy in a written design before connecting it to policy execution. Physical acceptance remains pending.
3. **Backend and configuration.** Add an inert six-value RoArm adapter, explicit identities/roles, calibrated limits and timeout settings. Use an injected transport in offline tests. Cover partial startup, disconnect, stale/replayed state, malformed lines, timeouts and exclusive ownership. Do not automatically home or release torque on import/connect.
4. **Collection and inference integration.** Remove ARX dimension/port assumptions only at the affected boundaries. Verify six/twelve-value metadata, saved poses, gripper angles, action targets and shared capture/recording/image parity. Run the upstream contract and affected configuration/runtime suites with simulated hardware.
5. **Target installation and measured acceptance.** Identify target OS/compute/cameras, build dependencies and firmware without flashing first, then run the operator checks in VALIDATION.md. Choose rate and action-step bounds from measurements.
6. **First learned task.** Collect, inspect and train one simple task; evaluate conservative execution and repeatability before smoothing/speed experiments. Keep task demonstrations, checkpoints and large logs outside this preparation folder, using the upstream storage-path configuration.
