# Findings and evidence

Access date for all sources: 2026-09-17. Labels distinguish vendor statements, inspected code, local checks and proposed design. No model capability was tested live.

## Jev: what is established

**J1 — Vendor documentation:** [State](https://docs.typesafe.ai/concepts/state). Jev currently accepts text/JSON, not images, audio or video. Robotics therefore needs a separate perception system.

**J2 — Vendor documentation:** [Introduction](https://docs.typesafe.ai/introduction), [API](https://docs.typesafe.ai/api), [Choice](https://docs.typesafe.ai/primitives/choice). Inputs are state plus typed questions; outputs include choices/distributions, scores, or yes/no probabilities. Questions in a batch are independent: one answer cannot feed another inside that call. Question IDs are routing labels, not instructions to the model; put the actual question in instructions/criteria.

**J3 — Vendor documentation, dated September 16:** [Jev 1.13 limitations](https://docs.typesafe.ai/model-jaggedness/jev-1.13). Numerical precision, counting, indirection and irrelevant context are documented weaknesses. Compute transforms, distances, counts and limits in code. These limitations are version-specific; do not assume preview is identical.

**J4 — Vendor documentation:** [Confidence](https://docs.typesafe.ai/confidence). Confidence is derived from the answer distribution, not an independent measurement of physical reliability. Noul does not return this confidence field. Any operational threshold needs held-out evaluation; a confident choice can still be wrong.

**J5 — Vendor documentation, mutable:** [Models](https://docs.typesafe.ai/models). The fetched page names `jev-1.13.0`, gives input-token pricing and warns rate limits can change. These are vendor terms, not measured run cost or latency. A dated alias/model record is required for experiments.

**J6 — SDK documentation:** [Constants](https://docs.typesafe.ai/sdk/python/api/constants), [Retries](https://docs.typesafe.ai/sdk/python/api/retries). The documented default HTTP-operation timeout is 10 seconds, and retries/backoff exist. That is not a robotics decision deadline. Wrap calls in a total deadline and discard expired decisions; never let retry behavior extend motion authority.

## Doom: model versus harness

**D1 — Official launch post:** [Introducing System One Models & Jev](https://typesafe.ai/blog/introducing-system-one-models-and-jev), September 15. TypeSafe explicitly says the demo consumes structured game state, not pixels, and says a conventional bot could play better. The post promises a future walkthrough. It does not establish physical manipulation ability. Reported speed/cost are vendor demonstration results, not our budget or benchmark.

**D2 — Independent public implementation inspected:** [lukaske/jev-doom-agent](https://github.com/lukaske/jev-doom-agent). This is NOT established as the official launch harness. Files inspected: README, `src/types.ts`, `src/policy.ts`, `src/doom.ts`, `server/typesafe.ts`.

The engine bridge supplies position, entities, geometry and other state. Local code computes spatial features and navigation behavior. The current server asks separate Choice questions for movement, view, trigger and interaction; a controller turns the answers into game inputs. This is substantial domain scaffolding. A robot has no equivalent authoritative engine state: camera calibration, object tracking and feedback must supply it.

The README's single-macro description is less specific than current four-axis server code. The server creates a one-hot aggregate action distribution, so that aggregate is not the raw Jev probability output. Preserve raw responses in our design. Do not transplant game-specific fallback or timing assumptions. These are inspection findings, not execution results.

**D3 — Search boundary:** Public searches for Jev Doom/robotics, the TypeSafe public repository list, and its demo index did not establish an official Doom source release or a public Jev/RoArm implementation. This does not prove either is absent, and does not establish first-mover status. The search was performed on September 17, 2026; absence from results is not proof of absence.

## Reusable architecture and prior art

**H1 — Independent Jev harness:** [browser-use/jev-ultrafast](https://github.com/browser-use/jev-ultrafast). README and `model.py`/`agent.py` inspected. It constructs choices from observed targets, selects an operation plus compatible target, and validates freshness before execution. Transferable lesson: offer only executable candidates and bind each decision to its observation. Browser timings are not robotics evidence.

**H2 — Robotics research:** [SayCan](https://say-can.github.io/). Language-conditioned selection among grounded robot skills predates Jev. Its separation between task relevance and executable skills is useful prior art. Our hypothesis is that Jev can fill part of the selection/recovery role efficiently; the architecture itself is not novel. No claimed first-mover win follows from early API access.

**H3 — Perception building block:** [OpenCV ArUco documentation](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html). Marker identity/corners and calibrated pose estimation provide a tractable starting point for a constrained table. Marker detection does not establish object graspability or successful placement. Real objects, camera calibration and arm-frame registration still require validation.

## Your RoArm evidence

**R1 — Deployment record:** [operating notes](https://github.com/jhacksman/RoArm-M3/blob/main/deployments/2026-09-08-m3-pro/docs/OPERATING_NOTES.md). The repo records two working leader/follower pairs, L1/F1 and L2/F2, on 0.84-s1, with user-observed powered tracking. This is historical evidence, not a fresh hardware inspection.

**R2 — Firmware guard scope:** [firmware fix](https://github.com/jhacksman/RoArm-M3/blob/main/deployments/2026-09-08-m3-pro/docs/FIRMWARE-SAFETY-FIX.md). The fix rejects invalid/stale leader feedback before ESP-NOW motion transmission. It explicitly is not a receiver watchdog or emergency-stop system, and does not eliminate startup motion. Its `motion_allowed` telemetry must not be interpreted as comprehensive robot safety approval.

**R3 — Inspected firmware definitions:** [json_cmd.h](https://github.com/jhacksman/RoArm-M3/blob/main/deployments/2026-09-08-m3-pro/src/RoArm-M3_safe/json_cmd.h) defines numeric `T` commands, including feedback and distinct radian/degree control families. Comments/constants are not a substitute for confirming dispatch behavior and deployed firmware.

**R4 — Local non-executing check:** `compile(source, filename, 'exec')` on the saved `lerobot/examples/pick_and_place.py` raises `SyntaxError: name 'ARM_IP' is used prior to global declaration`, line 155. Its string `type: AngleCtrl` calls also differ from the saved firmware definitions. It uses sleeps rather than verified action completion and attempts home motion during cleanup. Do not treat this example as a working skill library.

**R5 — Inspected imitation example:** `deploy_imitation_model.py` expects external model weights/metadata and uses similar string commands. It is not proof of a deployed trained policy. Existing leader/follower capability is a route to demonstrations; learned skills should follow a validated recording/actuation interface.

Waveshare wiki fetches failed. A guessed LeRobot RoArm page also failed. Do not infer supported upstream RoArm integration from those failures or from this repository's LeRobot directory name.
