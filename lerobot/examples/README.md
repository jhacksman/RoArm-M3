# LeRobot Examples for RoArm-M3 Pro

This directory contains example code and projects demonstrating the integration of the RoArm-M3 Pro robotic arm with the LeRobot AI framework. These examples are designed to help you understand how to use LeRobot with the RoArm-M3 Pro and serve as a starting point for your own projects.

## Example Overview

The examples are organized by complexity and purpose:

1. **Basic Control**: Simple examples demonstrating basic control of the RoArm-M3 Pro using LeRobot
2. **Data Collection**: Examples showing how to collect datasets for training
3. **Model Training**: Examples of training models on collected data
4. **Deployment**: Examples of deploying trained models to the RoArm-M3 Pro
5. **Advanced Applications**: More complex examples demonstrating advanced capabilities

## Prerequisites

Before running these examples, make sure you have:

1. Assembled and configured your RoArm-M3 Pro robotic arm
2. Installed LeRobot and its dependencies (see [Installation Guide](../interface/installation.md))
3. Configured the connection between LeRobot and your arm (see [Interface Documentation](../interface/README.md))

## Example List

### 1. Basic Control

- [**basic_http_control.py**](./basic_http_control.py): Control the RoArm-M3 Pro using HTTP communication
- [**basic_serial_control.py**](./basic_serial_control.py): Control the RoArm-M3 Pro using serial communication
- [**joint_position_control.py**](./joint_position_control.py): Control individual joint positions

### 2. Data Collection

- [**record_teleoperation.py**](./record_teleoperation.py): Record arm movements during teleoperation
- [**record_with_camera.py**](./record_with_camera.py): Record arm movements with synchronized camera data

### 3. Model Training

- [**train_imitation_model.py**](./train_imitation_model.py): Train an imitation learning model
- [**train_reinforcement_model.py**](./train_reinforcement_model.py): Train a reinforcement learning model

### 4. Deployment

- [**deploy_imitation_model.py**](./deploy_imitation_model.py): Deploy a trained imitation learning model
- [**deploy_with_safety.py**](./deploy_with_safety.py): Deploy a model with safety constraints

### 5. Advanced Applications

- [**pick_and_place.py**](./pick_and_place.py): Implement a pick-and-place task
- [**follow_trajectory.py**](./follow_trajectory.py): Follow a predefined trajectory
- [**visual_servoing.py**](./visual_servoing.py): Implement visual servoing with a camera

## Running the Examples

To run an example:

1. Connect to your RoArm-M3 Pro (via WiFi or USB)
2. Navigate to the examples directory
3. Run the example script:

```bash
# Activate your virtual environment
source lerobot_env/bin/activate  # On Windows: lerobot_env\Scripts\activate

# Run an example
python basic_http_control.py
```

## Modifying the Examples

Feel free to modify these examples to suit your needs. The examples are designed to be simple and easy to understand, providing a foundation for your own projects.

When modifying:

1. Start with the simplest example that's closest to your goal
2. Make incremental changes and test frequently
3. Refer to the [Interface Documentation](../interface/README.md) for details on available functions and parameters

## Troubleshooting

If you encounter issues with the examples:

1. Check the [Troubleshooting section](../interface/README.md#troubleshooting) in the Interface Documentation
2. Verify your connection to the RoArm-M3 Pro
3. Ensure you have the correct dependencies installed
4. Check the console output for error messages

## Contributing

If you develop new examples or improve existing ones, consider contributing them back to the community. Your contributions can help others learn and build with the RoArm-M3 Pro and LeRobot.
