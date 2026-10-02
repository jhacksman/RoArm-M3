# Dual RoArm-M3 development

**Status: offline preparation, 2026-09-30.** This wing prepares one Jetson to coordinate two RoArm-M3 arms. It contains a design, a pinned official asset inventory, a local asset fetch/check tool, and bounded implementation gates. It does not contain a working dual-arm controller, calibrated cell, validated simulator scene, or permission to move hardware.

Start with [architecture and host choice](ARCHITECTURE.md), [hardware and protocol constraints](HARDWARE.md), [asset provenance](assets/README.md), and [milestones](PLAN.md).

## What exists already

- The [September deployment](../deployments/2026-09-08-m3-pro/README.md) records four M3 Pro arms on guarded **0.84-s1**, with user-tested L1 → F1 and L2 → F2 tracking. That establishes the earlier demonstration setup, not autonomous dual-arm readiness. **Autonomous arm selection and eventual names are TBD.** F1/F2 mean follower roles in the existing teleoperation setup; they are not assigned autonomous targets. Preserve the L1 → F1 and L2 → F2 teleoperation labels and pairings.
- [Jev research](../experiments/jev/README.md) already separates task choices from local execution. [PR #20](https://github.com/jhacksman/RoArm-M3/pull/20), inspected at `1553d15a76c0829a8fdd417f326442fbc65102d2`, adds offline choice validation and synthetic replay. It remains separate and unmerged. Reuse its snapshots/freshness/replay semantics after review; do not fork another decision harness here.
- [Earlier Isaac Sim material](../isaac_sim/README.md) contains exploratory examples, not a verified M3 model or deployment recipe. Use this wing's dated requirements before choosing a simulator host.
- The inspected main tree (`0e801e0643d945f0674400414bb37a8cb6fd11d2`) contained no actual arm STEP/STL/URDF/Xacro files. The official sources now have a reproducible [manifest](assets/manifest.json).

## Offline starting point

From the repository root, with Python 3.10+ and no extra packages:

```sh
python3 dual_arm/tools/assets.py fetch
python3 dual_arm/tools/assets.py verify
python3 dual_arm/tools/assets.py inspect
python3 -m unittest discover -s dual_arm/tests -v
```

`fetch` downloads only the manifest's official reference files into ignored `dual_arm/assets/cache/`; it never extracts archives, installs software, contacts a robot, or sends commands. Existing files must match before reuse. Hash mismatch is a failure, never an automatic manifest update. Review source terms before redistributing any cached file. `verify` works offline; `inspect` checks model dependencies and reports known model limitations, not simulation validity.

## Recommendation

Start with **offline two-arm geometry and MoveIt 2 planning**, then a mock transport and failure replay. Use the existing Thor as the likely deployment candidate only after its storage and environment are checked; an Orin Nano remains a credible lighter controller/perception option subject to benchmarks. Keep Isaac Sim optional on a supported separate host for cameras, contacts, and synthetic data. Choose no hardware or software upgrade merely from peak AI specifications.

Potential later utility: material/media loading for the donated Snapmaker 2.0 and the speculative mkrbox manufacturing idea. That requires a separate machine-state/interlock/tooling design after the tabletop baseline. This change does not integrate or operate that machine.
