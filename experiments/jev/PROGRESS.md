# Progress

Updated September 17, 2026. Research/design complete for the initial pass; implementation not started.

Completed: primary Jev docs reviewed; independent Doom and browser harness source inspected; SayCan prior art reviewed; RoArm deployment notes/command definitions examined; old pick/place example syntax failure confirmed without executing its code. Findings distinguish source claims from measurements.

Deliverables: harness proposal, gated evaluation plan, synthetic API request and task backlog.

Unknowns: actual camera/mounting and hardware readiness; currently installed firmware; exact account-accessible model and API quota; measured latency; calibration and grasp reliability; independent task success; official Doom source provenance.

Next: RJ-01 platform/protocol audit and offline RJ-04 adapter design. No inference, motor command or firmware change has been performed by this experiment. Research is published for review in PR #19; it is not a controller release.

Claim a task ID with an owner and timestamp before implementation; preserve evidence and record failures. Test counts, timing budgets and acceptance thresholds must be declared as proposed or measured, never invented as achieved results.

September 17 sensor-fusion follow-up: documented aggregate flow versus wrist-camera tracking, overhead RGB-D roles, calibration/rotation/depth ambiguities, bounded reactive steering and a non-contact alignment experiment. Added RJ-09. Primary sources: TypeSafe, PX4, Bitcraze, OpenCV and multi-camera visual-servoing research. Proposed architecture only; no sensor purchase, credentials, inference or hardware execution. Next offline work can define RJ-09 while RJ-01 inventories actual hardware.

September 17 Thor follow-up: user reports Thor access. Added THOR_AND_JEV.md with six primary research references, versioned NVIDIA integration evidence, hybrid runtime design and measurement gates. Added RJ-10/RJ-11. Exact host and software remain uninspected. Local Jev weights/runtime availability is unverified. Research update targets https://github.com/jhacksman/RoArm-M3/pull/19; no hardware or cloud-model execution.
