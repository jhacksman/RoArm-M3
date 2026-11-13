# Agent handoff: Flash ALOHA for RoArm-M3 Pro

Prepared November 12, 2026. This folder is the result of the requested offline preparation. Physical integration is a subsequent task.

## User intent and decisions

Jack wants to reuse Flash ALOHA's learning pipeline with his Waveshare RoArm-M3 Pro setup. The immediate deliverable is a folder in this repository containing useful preparation that requires no hands-on access, ready to transfer to another agent later. The user corrected an earlier answer: the target is **not the Mac used to prepare this folder**. Do not restart Mac camera/USB setup or require a hostname to understand this package. The target OS, compute hardware, cameras and calibrated limits remain intentionally unspecified.

## What is ready

- Original project/code identified and reference revisions pinned in `sources.json`.
- Six-joint ordering and numeric JSON command format mapped from the deployed firmware source.
- A saved-feedback inspector and pure JSON decoder/target formatter, with no SDK or device I/O.
- A synthetic fixture, regression suite and source-drift checker using Python's standard library.
- An implementation map covering factory/configuration, recording, training, inference, gripper units, timing and failure behavior.
- A telemetry proposal, control-ownership alternatives and explicit hardware acceptance criteria.

The existing two leader/follower pairs were documented as user-tested in September 2026. That historical result does not establish that a learned policy or new adapter works. There is no RoArm Flash ALOHA backend, synchronized training dataset or trained checkpoint in this package. No firmware was changed or flashed during preparation.

## Resume immediately

From the RoArm repository root:

```sh
python3 lerobot/flash_aloha/run_offline_checks.py
python3 lerobot/flash_aloha/inspect_setup.py --feedback \
  lerobot/flash_aloha/fixtures/position.synthetic.json
```

Both should exit 0; the synthetic fixture still reports `robot_deployment_ready: false`, missing velocity and unverified freshness. The fixture's pose, voltage and any limits used in tests are illustrative. They are not operating settings.

Check out the pinned Flash ALOHA revision in a suitable development directory, then verify the supplied reference roots:

```sh
git clone https://github.com/JohnYao93/flash-aloha.git flash-aloha
git -C flash-aloha checkout 4172a8ed8cadb6eeb4a59285826e83cb00a2f01c
python3 lerobot/flash_aloha/verify_sources.py \
  --flash-root flash-aloha --roarm-root .
```

Use the manifest to find the exact official SDK revision if consulting its API. The source checker reads bytes only; it does not import code, fetch missing files or operate hardware. A mismatch means reassess the source map against that checkout before porting. A sparse checkout must include the listed files.

## Next implementation steps

Read `IMPLEMENTATION.md` and the applicable repository instructions. First resolve the missing state fields and stop/ownership design in reviewable source changes and simulated tests. Then extend all the backend, task, recorder and inference boundaries listed in the source map; changing only the arm factory will not work. Preserve the existing ARX paths and shared capture/image contracts.

When hardware is available, fill `target.example.json` in a working copy with measured/verified values and follow `VALIDATION.md`. Start with one pair and one simple soft-object task. Hardware identities, power-up sequence, backups and guarded firmware are already documented under `deployments/2026-09-08-m3-pro/`; read those notes before proposing device actions.

The Python packet formatter is an offline building block. A production adapter still needs exclusive serial ownership, framing/response association, real feedback age, velocity provenance, measured limits, rate/step constraints, startup anchoring, a target-specific watchdog/stop implementation and cleanup tests. A formatted JSON line is not permission or evidence to execute it.

## Copyable task for a later agent

> Continue the RoArm-M3 Pro Flash ALOHA integration from `lerobot/flash_aloha/HANDOFF.md`. Read repository AGENTS instructions, run the offline suite, and verify the pinned source references. This folder completes preparation only. Target hardware is another computer, with its details deferred. Implement and test the remaining state, stop, backend/configuration, recorder and inference changes against the documented contracts. Do not treat the pure protocol helpers or synthetic fixture as a live adapter or physical validation. Preserve 0.84-s1 guards and existing backups, and record measured evidence for every hardware acceptance item before claiming operation. Maintain the existing private knowledge-wiki work record rather than creating a duplicate for this preparation; create a distinct integration task only if a new assignment is confirmed.

## Transfer

Transfer this repository branch, or archive this folder together with access to the referenced RoArm deployment files. The offline unit suite is self-contained in the folder; source verification additionally requires the listed source checkouts. No absolute path from the preparation machine is required by the tools. Keep credentials, SSH configuration, recorded user data, model weights and large upstream checkouts outside this folder.
