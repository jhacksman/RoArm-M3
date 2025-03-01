# Domain Randomization for RoArm-M3 Pro

This directory contains examples and utilities for implementing domain randomization techniques with the RoArm-M3 Pro robotic arm in NVIDIA Isaac Sim.

## What is Domain Randomization?

Domain randomization is a technique used to bridge the reality gap between simulation and the real world. By randomizing various aspects of the simulation environment, policies trained in simulation become more robust and can better transfer to the physical robot.

## Contents

- `domain_randomization_example.py`: Example script demonstrating how to implement domain randomization for the RoArm-M3 Pro
- `README.md`: This file

## Key Randomization Techniques

The example demonstrates several key randomization techniques:

1. **Dynamics Randomization**: Varying physical properties like damping, stiffness, and friction
2. **Observation Noise**: Adding realistic sensor noise to joint position readings
3. **Visual Randomization**: Changing lighting conditions and visual appearance
4. **Mass Properties**: Randomizing mass and inertia of robot links

## Usage

To use the domain randomization example:

1. Ensure you have NVIDIA Isaac Sim installed
2. Update the `robot_usd_path` variable in the script to point to your RoArm-M3 Pro USD file
3. Run the script from Isaac Sim's Python environment:

```bash
cd ${ISAAC_SIM_PATH}
./python.sh /path/to/domain_randomization_example.py
```

## Integration with Training

When training policies for sim-to-real transfer, integrate domain randomization as follows:

1. Create a training environment that inherits from the `DomainRandomization` class
2. Apply randomization at the start of each episode
3. Use the randomized environment for policy training
4. Test the trained policy on the physical RoArm-M3 Pro

## References

- [OpenAI: Solving Rubik's Cube with a Robot Hand](https://openai.com/blog/solving-rubiks-cube/)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
