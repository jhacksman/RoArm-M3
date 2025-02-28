# Isaac Sim Examples for RoArm-M3 Pro

This directory contains example code demonstrating the integration of the RoArm-M3 Pro robotic arm with NVIDIA Isaac Sim. These examples are designed to help you understand how to use Isaac Sim with the RoArm-M3 Pro and serve as a starting point for your own projects.

## Example Overview

The examples are organized by complexity and purpose:

1. **Basic Simulation**: Simple examples demonstrating basic setup and control
2. **Data Collection**: Examples showing how to collect synthetic data
3. **Model Training**: Examples of training models in simulation
4. **Sim-to-Real**: Examples of transferring from simulation to the real arm
5. **Advanced Applications**: More complex examples demonstrating advanced capabilities

## Prerequisites

Before running these examples, make sure you have:

1. Installed NVIDIA Isaac Sim (see [Installation Guide](../interface/installation.md))
2. Created a digital twin of the RoArm-M3 Pro (see [Model Creation Guide](../interface/model_creation.md))
3. Set up the simulation environment (see [Simulation Environment Guide](../interface/simulation_environment.md))

## Example List

### 1. Basic Simulation

- [**basic_setup.py**](./basic_setup.py): Set up a basic simulation environment with the RoArm-M3 Pro
- [**joint_control.py**](./joint_control.py): Control individual joint positions in simulation
- [**inverse_kinematics.py**](./inverse_kinematics.py): Use inverse kinematics to control the arm

### 2. Data Collection

- [**collect_joint_data.py**](./collect_joint_data.py): Collect joint position data from simulation
- [**collect_visual_data.py**](./collect_visual_data.py): Collect RGB and depth images from simulation
- [**synthetic_dataset_generation.py**](./synthetic_dataset_generation.py): Generate synthetic datasets with domain randomization

### 3. Model Training

- [**train_rl_model.py**](./train_rl_model.py): Train a reinforcement learning model in simulation
- [**train_imitation_model.py**](./train_imitation_model.py): Train an imitation learning model in simulation
- [**train_with_domain_randomization.py**](./train_with_domain_randomization.py): Train with domain randomization

### 4. Sim-to-Real

- [**sim_to_real_calibration.py**](./sim_to_real_calibration.py): Calibrate simulation to match the real arm
- [**deploy_sim_model.py**](./deploy_sim_model.py): Deploy a model trained in simulation to the real arm
- [**reality_gap_analysis.py**](./reality_gap_analysis.py): Analyze and measure the reality gap

### 5. Advanced Applications

- [**pick_and_place.py**](./pick_and_place.py): Implement a pick-and-place task
- [**object_sorting.py**](./object_sorting.py): Sort objects by type
- [**visual_servoing.py**](./visual_servoing.py): Implement visual servoing with a camera

## Running the Examples

To run an example:

1. Launch Isaac Sim
2. Open the Python script editor
3. Load the example script
4. Run the script

Alternatively, you can run the examples from the command line:

```bash
# Navigate to Isaac Sim installation directory
cd /path/to/isaac-sim

# Run an example
./python.sh /path/to/RoArm-M3/isaac_sim/examples/basic_setup.py
```

## Modifying the Examples

Feel free to modify these examples to suit your needs. The examples are designed to be simple and easy to understand, providing a foundation for your own projects.

When modifying:

1. Start with the simplest example that's closest to your goal
2. Make incremental changes and test frequently
3. Refer to the [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html) for details on available functions and parameters

## Hardware Requirements

These examples have varying hardware requirements:

- **Basic Simulation**: Minimal requirements (RTX 2070 or equivalent)
- **Data Collection**: Moderate requirements (RTX 3070 or equivalent)
- **Model Training**: High requirements (RTX 3080/3090 or equivalent)
- **Advanced Applications**: High requirements with 16GB+ VRAM

For optimal performance, especially when training complex models, we recommend:
- NVIDIA RTX 3090 or better
- 64GB system RAM
- 24GB+ VRAM

## Troubleshooting

If you encounter issues with the examples:

1. Check the [Troubleshooting section](../interface/installation.md#troubleshooting) in the Installation Guide
2. Verify your Isaac Sim installation
3. Ensure your hardware meets the requirements
4. Check the console output for error messages

## Contributing

If you develop new examples or improve existing ones, consider contributing them back to the community. Your contributions can help others learn and build with the RoArm-M3 Pro and Isaac Sim.
