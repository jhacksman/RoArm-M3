# Official asset inventory and provenance

Discovered from the [Waveshare RoArm-M3 resource links](https://www.waveshare.com/wiki/RoArm-M3), retrieved 2026-09-30. [manifest.json](manifest.json) records exact URLs, sizes and SHA-256 hashes for **15 files / 8,275,926 bytes**. Hashes establish repeatability of the retrieved bytes, not independently signed vendor authenticity.

| Source | Gathered local files | Status |
| --- | --- | --- |
| [Official ROS workspace](https://github.com/waveshareteam/roarm_ws/tree/40dbd84b553695212fab713e8465f817ba95454d/src/roarm_main/roarm_description), `ros2-humble`, pinned commit `40dbd84b553695212fab713e8465f817ba95454d` | `cache/roarm_description/package.xml`; four files under `urdf/roarm_m3/`; eight STL files under `meshes/roarm_m3/` | Original bytes, no renames/conversion. Main Xacro references seven meshes; `gripper_left_link.stl` is also gathered but not referenced there. This is a description subset, not a buildable ROS workspace. |
| [STEP archive](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_STEP_260310.zip) | `cache/RoArm-M3_STEP_260310.zip` | Contains `RoArm-M3_STEP/RoArm-M3.step`, 26,671,289 uncompressed bytes. Archive retained, no extraction by tool. |
| [2D dimensions archive](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_2Dsize.zip) | `cache/RoArm-M3_2Dsize.zip` | Contains `RoArm-M3_size.pdf`. Drawing is a reference, not confirmation of the installed hardware revision. |

## License boundary

The official `roarm_description/package.xml` declares **MIT**. No package/root LICENSE file was found in the inspected vendor tree. Preserve the package metadata and upstream identity; do not invent a copyright holder or apply this repository's documentation license to third-party content. Neither CAD archive includes a license file. Public download availability does not establish redistribution rights.

Accordingly **all retrieved third-party bytes remain in ignored `cache/`**, available locally for inspection; this PR commits only provenance, original documentation and tooling. Resolve source attribution/license text and CAD redistribution permission before vendoring files, derived meshes or a published USD scene. Future transformations must record input hashes, conversion tool/version/options, changed limits/inertias and output hashes. Do not silently refresh mutable download URLs.

## Observed model gaps

`tools/assets.py inspect` performs static checks only. At the pinned commit:

- The model has five pose revolute joints plus one gripper joint, two fixed joints, nine links including world/TCP, and seven referenced mesh files. The mesh scale is `0.001` on each axis.
- All six revolute joints declare zero velocity and effort limits. Mass/inertia values exist but have not been checked against the physical Pro arms.
- Names and the `world_to_base_link` fixed transform are hardcoded. A dual model needs prefixes and measured base transforms; concatenating two copies causes name collisions.
- Includes use `$(find roarm_description)`; meshes use `file://$(find roarm_description)`. Resolve these during a controlled Xacro expansion/export step before import. This subset is not a standalone URDF.
- `.trans` uses legacy `hardware_interface/EffortJointInterface`; `.gazebo` references `libgazebo_ros_control.so`. These are not a tested modern ROS 2/Gazebo control integration.
- The left gripper mesh is absent from the main model's references; review physical gripper geometry/collision coverage before planning.

No STEP conversion, Xacro expansion, collision simplification, simulator import or dynamics validation has been performed. The `verify`/`inspect` commands cannot certify model correctness or safe motion.
