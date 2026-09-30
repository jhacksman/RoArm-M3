# Bounded next milestones

All stages are pending unless explicitly marked complete. Each stage should be one focused follow-up change, with its evidence checked before the next.

| Gate | Deliverable | Acceptance evidence |
| --- | --- | --- |
| DA-00 — complete | Separate development wing, current official source manifest, local asset cache, architecture and protocol audit | SHA-256/size checks, XML dependency inspection and documentation review; no claim of robot or simulator execution |
| DA-01 — next | Confirm arm pair/revisions, cell measurements and task; record calibration worksheet with unknowns explicit | User-confirmed target IDs; measured layout/TCP; selected lightweight part and two disjoint work zones; payload and stop design recorded |
| DA-02 | Import-ready single-arm description, then namespaced dual-arm composition | Resolve all meshes/includes; verify scale and FK against dimensions; replace zero dynamics limits with sourced values; review gripper and shoulder mapping; one world, unique names, table and inter-arm collisions; no hardware transport |
| DA-03 | Offline MoveIt planning plus mock two-arm adapter | Successful/rejected plans in shared scene; joint bounds, collisions, missing/stale feedback, late/duplicate commands, wrong arm, disconnect, ownership conflict and restart tested; measure scheduling skew in replay |
| DA-04 — optional | Isaac Sim or Gazebo scene for a named camera/contact question | Supported pinned host/stack; imported asset audit; collision/inertia checks; two-arm namespace isolation; measured simulation timing; no physical bridge |
| DA-05 | Jetson recorded-data deployment benchmark | Reuse prior Thor inventory; confirm storage allocation and compatible stack without disrupting services; record CPU/GPU/RAM, perception latency, planner latency and control jitter under concurrent load; compare Nano only if available/needed |
| DA-06 — hardware approval gate | Reviewed adapter and supervised commissioning plan | Validated stop/ownership/fault behavior, physical stop and operator procedure, exact firmware/model/calibration hashes; user authorizes powered tests. Begin one arm at a time, then alternating two-arm motion in separate zones |
| DA-07 | Bounded two-arm tabletop task | Repeatable measured success/failure outcomes, collision clearances, latency and stop/restart evidence; only then evaluate shared-zone cooperation |

DA-03 may reuse PR20's choice/replay validation once reviewed and available; keep its `accepted_for_shadow` outcome separate from execution permission. This plan neither merges PR20 nor changes Jev's backlog. A deterministic task baseline comes before policy/model selection.

## Decisions needed from the owner

1. Which physical arms will be selected for autonomous control, and what are their verified hardware revisions? Selection and eventual names are TBD; F1/F2 remain follower labels for the existing teleoperation setup.
2. What first tabletop task, part mass and fixture layout should define DA-01? Provide measured mounting spacing/orientation when available.
3. Should the existing Thor be the initial deployment target, and is there a supported RTX workstation available for optional simulation? Orin Nano module/RAM matters if that is the preferred target.

These answers block hardware-specific configuration, not offline asset work. Camera selection, gripper/tool geometry, calibration and storage allocation remain explicit follow-up inputs. No simulator, ROS, CUDA stack, service or hardware probing is installed/started by this change.

## Deferred manufacturing use case

mkrbox-inspired material/media loading for the donated Snapmaker 2.0 (three quick-change attachments, setup pending) is a later design input. Each machine mode needs separate envelopes, fixture/tool clearance, material/payload handling, readiness/fault signals, interlocks and machine/robot ownership. The present milestone is a tabletop baseline; no machine connection, job control or autonomous loading is authorized or implemented.
