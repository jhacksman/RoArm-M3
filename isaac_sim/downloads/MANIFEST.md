# NVIDIA Isaac Sim Downloaded Resources for RoArm-M3 Pro

This manifest documents all the resources downloaded from NVIDIA Isaac Sim for integration with the RoArm-M3 Pro robotic arm.

## GitHub Repositories

### 1. IsaacSim-ros_workspaces
- **Description**: ROS integration examples for Isaac Sim
- **Source**: https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces
- **Purpose**: Provides examples and templates for integrating ROS/ROS2 with Isaac Sim, which is essential for connecting the simulation to the physical RoArm-M3 Pro

### 2. OmniIsaacGymEnvs
- **Description**: Reinforcement learning environments for Isaac Sim
- **Source**: https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs
- **Purpose**: Provides reinforcement learning environments and examples for training policies in Isaac Sim that can be transferred to the physical RoArm-M3 Pro

### 3. CuRobo
- **Description**: CUDA-accelerated robot motion planning library
- **Source**: https://github.com/NVlabs/curobo
- **Purpose**: Provides fast motion planning capabilities for robotic arms, which is essential for the sim-to-real self-improvement loop

## Examples

### CuRobo Examples
- **Description**: Robot arm manipulation examples using CuRobo
- **Source**: Extracted from https://github.com/NVlabs/curobo/tree/main/examples
- **Purpose**: Provides practical examples of motion planning, collision avoidance, and control for robotic arms

## Custom Sim-to-Real Resources

### 1. Domain Randomization
- **Description**: Examples and utilities for implementing domain randomization techniques
- **Purpose**: Helps bridge the reality gap by making policies robust to variations in physical parameters
- **Files**:
  - `domain_randomization_example.py`: Example script for implementing domain randomization
  - `README.md`: Documentation on domain randomization techniques

### 2. Calibration
- **Description**: Tools for calibrating simulation parameters to match the physical robot
- **Purpose**: Reduces the reality gap by making the simulation behave more like the physical robot
- **Files**:
  - `calibration_example.py`: Example script for calibrating simulation parameters
  - `README.md`: Documentation on calibration techniques

### 3. System Identification
- **Description**: Methods for identifying system parameters of the physical robot
- **Purpose**: Enables accurate modeling of the physical robot in simulation
- **Files**:
  - `system_identification_example.py`: Example script for identifying system parameters
  - `README.md`: Documentation on system identification techniques

## Hardware Requirements and Limitations

- **Description**: Documentation on hardware requirements and limitations for running Isaac Sim with the RoArm-M3 Pro
- **Purpose**: Helps users understand the hardware constraints and optimize their setup
- **Files**:
  - `hardware_requirements.md`: Detailed information on hardware requirements and VRAM constraints
  - `troubleshooting.md`: Common issues and solutions when working with Isaac Sim
  - `external_resources.md`: Links to additional documentation and resources

## Usage Notes

These resources are intended to be used together to create a comprehensive sim-to-real self-improvement loop for the RoArm-M3 Pro robotic arm. The workflow typically involves:

1. Creating an accurate digital twin of the RoArm-M3 Pro in Isaac Sim
2. Calibrating the simulation parameters using system identification
3. Implementing domain randomization to make policies robust
4. Training policies in simulation using reinforcement learning
5. Transferring the trained policies to the physical robot
6. Collecting data from the physical robot to improve the simulation
7. Repeating the process to continuously improve performance

## License and Attribution

All resources from NVIDIA repositories are subject to their original licenses. Custom examples and documentation created for this project are provided under the same license as the RoArm-M3 Pro documentation repository.
