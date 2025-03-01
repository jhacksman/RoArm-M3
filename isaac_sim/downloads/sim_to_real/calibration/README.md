# Simulation Calibration for RoArm-M3 Pro

This directory contains examples and utilities for calibrating simulation parameters to match the physical RoArm-M3 Pro robotic arm in NVIDIA Isaac Sim.

## What is Simulation Calibration?

Simulation calibration is the process of adjusting simulation parameters to make the simulated robot behave as closely as possible to the physical robot. This is a critical step in the sim-to-real transfer learning pipeline, as it helps reduce the reality gap.

## Contents

- `calibration_example.py`: Example script demonstrating how to calibrate simulation parameters for the RoArm-M3 Pro
- `README.md`: This file

## Key Calibration Parameters

The example demonstrates calibration of several key parameters:

1. **Joint Damping**: Controls how quickly joint motion is damped
2. **Joint Stiffness**: Controls the stiffness of position control
3. **Joint Friction**: Controls the friction in the joints
4. **Mass Scale**: Scaling factor for the mass of robot links

## Calibration Process

The calibration process involves the following steps:

1. Connect to the physical RoArm-M3 Pro robot
2. Generate a test trajectory that covers the robot's workspace
3. Execute the trajectory on the physical robot and record data
4. Simulate the same trajectory with different parameter sets
5. Optimize parameters to minimize the difference between simulation and reality
6. Apply the optimized parameters to the simulation
7. Validate the calibration with new trajectories

## Usage

To use the calibration example:

1. Ensure you have NVIDIA Isaac Sim installed
2. Connect your RoArm-M3 Pro to your computer
3. Update the `robot_usd_path` and `real_robot_ip` variables in the script
4. Run the script from Isaac Sim's Python environment:

```bash
cd ${ISAAC_SIM_PATH}
./python.sh /path/to/calibration_example.py
```

## Integration with Training

After calibrating your simulation:

1. Save the calibrated parameters using `save_calibrated_parameters()`
2. Load these parameters at the start of your training scripts
3. Apply the parameters to your simulation environment
4. Train your policies in the calibrated environment
5. Deploy the trained policies to the physical RoArm-M3 Pro

## References

- [System Identification for Sim-to-Real Transfer](https://arxiv.org/abs/1911.09256)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
