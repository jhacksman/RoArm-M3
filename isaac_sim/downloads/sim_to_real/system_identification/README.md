# System Identification for RoArm-M3 Pro

This directory contains examples and utilities for identifying system parameters of the physical RoArm-M3 Pro robotic arm for accurate simulation in NVIDIA Isaac Sim.

## What is System Identification?

System identification is the process of building mathematical models of dynamic systems based on observed input-output data. For robotics, this means determining parameters like mass, inertia, friction, and damping that govern the robot's behavior.

## Contents

- `system_identification_example.py`: Example script demonstrating how to identify system parameters for the RoArm-M3 Pro
- `README.md`: This file

## Key Parameters Identified

The example demonstrates identification of several key parameters:

1. **Mass**: The mass of each joint/link
2. **Damping**: The damping coefficient of each joint
3. **Friction**: The friction coefficient of each joint
4. **Gravity Coefficients**: Parameters related to gravity compensation

## Identification Process

The system identification process involves the following steps:

1. Connect to the physical RoArm-M3 Pro robot
2. Generate an identification trajectory that excites the system dynamics
3. Execute the trajectory on the physical robot and record data
4. Process the data to identify system parameters
5. Validate the identified parameters with a test trajectory
6. Export the parameters to a format usable by Isaac Sim

## Usage

To use the system identification example:

1. Ensure you have NVIDIA Isaac Sim installed
2. Connect your RoArm-M3 Pro to your computer
3. Update the `real_robot_ip` variable in the script
4. Run the script:

```bash
python system_identification_example.py
```

## Integration with Isaac Sim

After identifying system parameters:

1. Export the parameters using `export_parameters_to_isaac_sim()`
2. Load these parameters in your Isaac Sim environment
3. Apply the parameters to your simulated RoArm-M3 Pro
4. Validate that the simulation matches the physical robot's behavior

## References

- [System Identification for Control](https://en.wikipedia.org/wiki/System_identification)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
