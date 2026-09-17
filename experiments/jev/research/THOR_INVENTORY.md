# Thor inventory and first execution

Observed September 17, 2026 over authenticated SSH. User confirms robotics is the intended primary purpose; the existing H3 workload was a side project. Access details and raw machine records stay in the private fleet reference.

| Item | Observed result |
| --- | --- |
| Board | NVIDIA Jetson AGX Thor Developer Kit |
| Architecture / OS | aarch64 / Ubuntu 24.04.3 LTS |
| L4T | 38.4.0 |
| RAM | 122 GiB reported by Linux |
| CUDA toolkit package | 13.0.0-1 |
| TensorRT packages | 10.13.3.9 with CUDA 13.0 |
| System Python | 3.12.3 |
| Python module discovery | cv2, numpy, tensorrt present; torch and rclpy absent from system Python |
| ROS / Isaac | No matches in installed package query or /opt/ros; container/virtualenv contents not exhaustively audited |
| Camera devices | No /dev/video* entries observed; this does not exclude network or other camera interfaces |
| Root storage | 936 GiB total, 38 GiB available, 96% used |
| Current workload | Existing H3 container running; GPU utilization sampled at 0% |

Package presence is not proof of GPU execution or compatibility of a complete robotics stack. No JetPack metapackage version was established; do not infer it solely from the CUDA version. Storage capacity needs attention before large robotics images/models. No files, images or caches were deleted and no existing service was stopped.

## Offline harness validation

Uploaded six explicitly selected public files from source commit `041c0e9` into a new isolated workspace. All six SHA-256 hashes matched the source. Executed on Thor:

```sh
python3 -m unittest discover -s tests -v
python3 -m jev_replay examples/synthetic-replay.json
```

All 13 tests passed and all seven synthetic response attempts matched expected verdicts. These are CPU/software contract checks, not GPU benchmarks, live Jev inference, perception evaluation or robot trials. No dependencies were installed.

Next: choose a compatible pinned robotics environment after checking storage allocation, identify the actual cameras and control transport, and evaluate perception on recordings before physical execution. RJ-10 is partially complete: authenticated platform inventory and offline harness execution are done; perception replay and concurrent GPU profiling remain pending.
