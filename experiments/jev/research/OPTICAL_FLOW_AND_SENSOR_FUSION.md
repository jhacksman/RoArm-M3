# Wrist optical flow, overhead perception and Jev steering

Research/design update: September 17, 2026. No hardware, inference, latency or task-success measurements have been made. All architecture and experiments below are proposals.

## Answer and Doom analogy

Jev could choose short steering intentions repeatedly, rather than only choosing whole pick/place routines. The harness would supply a compact, grounded world state and a list of currently executable actions. Local code would implement and terminate each action, update perception and reject stale decisions. This is a plausible experiment, not demonstrated Jev robotics capability.

TypeSafe says its Doom example consumes structured game information; its documentation currently accepts text/JSON rather than camera images. A physical setup must reconstruct the state that a game engine already knows. The independently inspected Doom repository is not verified as the official demo. See FINDINGS.md for provenance.

A useful division is: sensors and estimation build the game state; local motion control implements the game physics and controls; Jev chooses what to do next. Do not make Jev integrate optical flow, calculate inverse kinematics or regulate servo timing.

## What the sensors contribute

| Input | Useful contribution | What it cannot establish alone |
| --- | --- | --- |
| Aggregate optical-flow module near each gripper | Local apparent image motion, motion consistency and stalls under suitable viewing conditions | Object identity, absolute position, depth, grasp success or whether image motion came from the wrist or object |
| Wrist camera with software flow and feature tracking | Target-relative image error, local feature motion and close-up observations | Metric depth without additional geometry/ranging; visibility through occlusion |
| Fixed overhead RGB-D camera | Shared scene frame, object locations and coarse depth, workspace context | All surfaces, hidden fingers, reliable depth on every material or continuous visibility during a grasp |
| Arm joint feedback | Kinematic prediction of wrist pose and camera rotation | Exact tool pose despite calibration error, mechanical play or external contact |

Optical flow is apparent motion in the image. Wrist rotation also creates it; translational scale depends on surface distance. A moving target and a moving sensor can produce ambiguous measurements. Texture, exposure, reflections, motion blur and which surface fills the field of view matter. Integrating flow alone accumulates error.

PX4's implementation illustrates the need for distance and rotation compensation when using flow for velocity estimation. Bitcraze pairs a PMW3901 flow sensor with a range sensor and notes matte-surface dependence. These are aerial-robot examples, not validation of the same modules on an arm. Check the exact module's working distance and optics against the intended gripper mounting before selecting it.

Two wrist modules do not automatically form stereo or share observations. Fuse each against its arm's calibrated frame and feedback, then associate objects in a common table frame. Use timestamps, quality/uncertainty and residual checks; do not average unrelated flow vectors. Moving-camera extrinsics, sensor offsets and time alignment are part of the work. An overhead depth measurement is only useful for wrist-flow scale if it refers to the corresponding observed surface and is still valid.

## Proposed control architecture

1. Acquire joint feedback and images/flow with capture timestamps and quality flags.
2. Calibrate intrinsics, wrist-to-camera and overhead-to-table/arm transforms. Check tool offsets and frame conventions.
3. Track targets and estimate relative tool/target state locally. Compensate camera motion using calibrated kinematics or appropriate measured rotation; reject inconsistent measurements. Preserve uncertainty and provenance rather than turning disagreement into a confident average.
4. Derive decision facts: target visible, left/right of alignment region, approaching desired clearance, alignment improving, motion unconfirmed, overhead occluded, other arm owns workspace, observation stale.
5. Construct feasible, bounded action candidates. Jev selects one action from the joint set; local code validates the choice against fresh observations and controls execution.
6. Observe the outcome and repeat. A local supervisor controls deadlines, action leases, single ownership and tested stop/hold behavior independently of cloud availability.

Illustrative actions: align with target, approach to a validated standoff, move to a known observation pose, wait for the other arm, grip when local preconditions are satisfied, and retreat along a validated route. An alternative experimental action set uses short target-relative translation steps whose direction, size and duration are computed locally. Never infer those quantities from labels alone.

The local visual servo can continuously reduce image error while Jev decides whether to continue, change target, seek another view or recover. Jev still steers the behavior. Complete pre-scripted task routines are not required, but tested motion primitives and geometric constraints are. Bimanual actions require arbitration and checks on both arms' swept volumes, not just tool separation.

Synthetic decision-state illustration (not an API request or sensor record):

```json
{
  "goal": "align above the marked target",
  "target": "marker_A",
  "wrist_tracking": "valid",
  "alignment": "target_left_of_tool",
  "recent_progress": "improving",
  "overhead_visibility": "temporarily_occluded",
  "flow_vs_predicted_motion": "consistent",
  "other_arm": "parked_outside_work_area",
  "available_actions": ["continue_alignment", "hold_and_reobserve"]
}
```

Every derived label needs an explicit, calibrated definition. Unknown remains unknown. The real contract also contains epochs, timestamps, calibration versions and immutable candidate IDs described in HARNESS.md. Retain numerical state locally for control, validation and evaluation even when Jev receives semantic summaries.

## How much harness?

The API adapter is a small portion. Acquisition, calibration, tracking/fusion, motion control, supervision and outcome verification are substantial reusable robotics work. Exact development effort depends on the actual camera setup and the existing controller; no defensible time estimate is available yet.

| Experiment scope | Required foundation |
| --- | --- |
| One arm aligning above a marked stationary target | Calibrated wrist image, target tracking, bounded motion, feedback, observation freshness and tested stop behavior |
| Target motion or temporary occlusion | Motion estimation, reacquisition, uncertainty handling and alternate observations |
| Two arms manipulating objects | Common scene frame, shared workspace arbitration, collision checks, grasp verification and coordination |

An overhead camera helps global context; RGB-D adds depth but also calibration and depth-validity work. Dedicated flow modules add hardware and fusion work. A wrist camera lets us first test software flow and target tracking on the same recorded images. This is an experimental sequencing recommendation, not a claim that a particular camera is compatible or sufficient.

## First experiment and evidence

Start with one arm maintaining alignment above a marked tabletop target at a validated standoff, without contact. Establish a local visual-servo baseline. Add Jev to select continuation, reacquisition and recovery actions. Introduce target movement and controlled occlusion only after stationary alignment works. Add contact and grasping afterward; add the second arm last.

Compare overhead-only observations, overhead plus wrist tracking, and the same setup with optical-flow features. Keep task and motion limits comparable. Compare Jev with a deterministic supervisor using the same sensor state and action vocabulary. Test changes in height, wrist orientation, texture, lighting and visibility; distinguish recorded replay from physical trials.

Record alignment error using an independent reference where feasible, reacquisition time, task outcome, interventions, sensor disagreement, stale decisions and end-to-end observation-to-action delay. Declare acceptance criteria before collecting comparison results. This determines whether flow helps and whether Jev contributes beyond ordinary visual servoing. No claimed winner yet.

## Sources and limits

- [TypeSafe state documentation](https://docs.typesafe.ai/concepts/state): current text/JSON input contract.
- [TypeSafe launch and Doom discussion](https://typesafe.ai/blog/introducing-system-one-models-and-jev): vendor description of the demo, not robotics validation.
- [PX4 optical flow](https://docs.px4.io/main/en/sensor/optical_flow): distance/rotation considerations for flow-based estimation.
- [Bitcraze Flow deck v2](https://www.bitcraze.io/products/flow-deck-v2/): flow/range pairing and surface considerations.
- [OpenCV tracking API](https://docs.opencv.org/4.13.0/dc/d6b/group__video__track.html): available software flow/feature-tracking primitives, not an implemented robot estimator.
- [Hybrid Multi-camera Visual Servoing to Moving Target, IROS 2018](https://arxiv.org/abs/1803.02285): prior work combining external and arm-mounted cameras with visibility-dependent control; its hardware differs from this proposal.

The proposed RoArm architecture and evaluation are our engineering hypotheses. Multi-camera visual servoing already has prior art; novelty or first-mover advantage for this particular Jev integration has not been established.
