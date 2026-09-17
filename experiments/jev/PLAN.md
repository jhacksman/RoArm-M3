# Build and evaluation plan

No schedule, revenue forecast or achieved success rate is asserted. Gates below are proposed engineering criteria. Camera, gripper condition, mounting, host, control transport and physical availability remain unknown.

## Gate A: establish the real platform

Inventory the actual arms and camera; compare installed firmware to the saved 0.84-s1 record. Validate controller dispatch and feedback without motion first. Establish command ownership so teleoperation and autonomy cannot drive the same follower. Determine actual stop behavior, communication-loss behavior, startup movement and operator access to power interruption. Inspect the existing sender guard separately from the needed host/receiver watchdog.

Deliverable: exact hardware/software inventory, non-motion feedback trace, calibration plan, tested stop procedure. No motor-enabling experiment should rely only on a browser playground result.

## Gate B: make one physical skill work without Jev

Mount the arm and camera; use an easy lightweight part and a known source/destination fixture. Validate one pick/place sequence with real feedback and visual outcome checks. Log failures and interventions, not just successful recordings. Replace the broken old examples with a protocol-verified adapter when implementation is requested.

Deliverable: repeatable observed skill with failure categories and a replayable trace. If this fails, improve mechanics/perception/control first; a decision model cannot repair an unreliable grasp primitive.

## Gate C: test Jev without motor authority

Use the synthetic request in `examples/decision-request.json` to establish request shape. Then feed recorded physical observations to Jev in shadow mode. Record model ID, raw answers, question revision, tokens, total latency and deadline misses. Compare suggestions to an independently labeled expected action; include correct abstention cases.

Do not infer API latency from the browser's rendering time. Separate perception, serialization, network/inference, validation and execution time. Record p50/p95/p99 only once an actual sample set exists. API budget must be calculated from observed token usage and actual account terms.

## Gate D: closed-loop supervised kitting

Allow only previously validated skills and candidate IDs. Re-observe after every completed action. Use a fixed, predeclared evaluation suite, randomized layout/task order, and matching conditions across policies:

- Deterministic recipe/state-machine baseline.
- Jev selecting among identical skills and observations.
- Optional other-model baseline later, without silently giving it better sensors or extra skills.

Scenario families: requested kit changes; irrelevant parts present; missing requested part; ambiguous reference; occupied destination; failed grasp; object moved after observation; camera occlusion; stale telemetry; delayed/failed API call; human intervention; duplicate completion report.

Record all attempts, actual contents of the tray, interventions, wrong-part selections, dropped objects, rejected actions, unhandled faults, cycle times and costs. Separate model failures from perception and actuator failures. Have an independent physical-outcome check; never score from Jev's own DONE answer. Any uncontrolled motion is a failed trial requiring investigation.

Predeclare a small pilot count and acceptance criteria before the experiment, then use a held-out batch before changing claims. There is no credible success percentage yet. A complete uncut run plus all-trial logs is stronger evidence than a highlights video.

## Gate E: earn a useful public claim

Publish only demonstrated capabilities: exact hardware, model/version, supported task, fixtures, dependencies, failures and reproduction instructions. Test varied instructions and conditions without per-test code changes. If Jev offers no benefit over the state machine, keep the robot working and reconsider Jev's role.

Being early can help visibility, but a defensible asset would be the working harness, task data, outcome checks and reproducibility. Research already predates Jev on grounded skill selection. Do not claim first unless the scope and prior-art search support it.

