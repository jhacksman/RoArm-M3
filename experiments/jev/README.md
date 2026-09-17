# Jev experiments for RoArm-M3 Pro

Research/design stage, September 17, 2026. No live model inference or physical robot trials have been performed for this experiment.

Investigate Jev as a task-level decision component around a locally controlled robot. Proposed first task: single-arm tabletop kitting of known lightweight parts from marked nests into a tray, followed by independently observed outcomes. The experiment must compare against a deterministic baseline using the same sensors and skills.

- [Research findings and primary sources](research/FINDINGS.md)
- [Optical flow, sensor fusion and Doom-style steering](research/OPTICAL_FLOW_AND_SENSOR_FUSION.md)
- [Jetson Thor plus Jev: papers and architecture](research/THOR_AND_JEV.md)
- [Harness design](HARNESS.md)
- [Build and evaluation gates](PLAN.md)
- [Progress](PROGRESS.md)
- [Claimable backlog](backlog.json)
- [Synthetic API request](examples/decision-request.json) — offline illustration; no motor authority

The model chooses from feasible task actions. Local code owns perception, numerical calculations, motion, freshness checks and stop behavior. Existing leader/follower pairs can supply demonstrations after the recording/control interface is validated.

## Public repository hygiene

Never commit API keys, tokens, credentials, account identifiers, browser exports, raw private logs or `.env` files. Use `TYPESAFE_API_KEY` supplied from a private environment/secret manager when a client is implemented. No key is required to read this research. Do not print authorization headers or serialize environment variables into traces.

Local run artifacts are ignored. Publish only deliberately reviewed/redacted traces, with model version and failure evidence. `.gitignore` is a convenience, not a secret scanner. No credentials or account screenshots are included here.

This is not a working controller, certified safety system, verified performance result, revenue claim or first-in-field claim.
