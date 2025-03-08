# RoArm-M3 Isaac Sim Bridge

## Overview

The RoArm-M3 Isaac Sim Bridge is a software solution that enables integration between the Waveshare RoArm-M3 robotic arm and NVIDIA Isaac Sim. This bridge facilitates data ingestion from the RoArm-M3 at 100Hz into Isaac Sim, with auto-provisioning for device identification and integration with camera feeds for sim2real training.

The RoArm-M3 Pro is a 5+1 DOF (Degrees of Freedom) smart robotic arm designed for innovative applications. It features a lightweight structure with an effective load capacity of 0.2kg when grabbing objects 0.5m away from the mechanical arm's origin. The Pro version differs from the standard M3 model by using metal shell servos (ST3235) for all joints except the gripper, providing reduced backlash that doesn't increase over time.

![RoArm-M3](https://www.waveshare.com/w/upload/8/8f/360px-RoArm-M3-S-2.jpg)

## Bridge Features

- **High-Frequency Data Ingestion**: Captures telemetry data from the RoArm-M3 at 100Hz
- **Auto-Provisioning**: Automatically identifies devices based on MAC address
- **Camera Integration**: Supports synchronized camera feeds for sim2real training
- **Device State Detection**: Intelligently detects different device states:
  - Normal Operation: All servos powered and responding
  - Unpowered Servos: ESP32 running but servos unpowered
  - Partial Operation: Some servos working, others not
  - Communication Failure: No servo communication
- **Plug-and-Play Compatibility**: Easy setup without manual configuration of COM ports
- **Extensible Architecture**: Modular design for adding new features and devices

## RoArm-M3 Key Features

- **Omnidirectional Workspace**: 360° all-round rotating base with flexible joint movement creates a working space with a diameter of up to 1m
- **Joint Direct-Drive Design**: Uses 12-bit high-precision magnetic encoder to obtain joint angle with repositioning accuracy up to 0.088°
- **Dual-Drive Technology**: Innovative dual-drive technology enhances the torque of the shoulder joint and improves overall loading capacity
- **Powerful Control Module**: Equipped with ESP32 MCU supporting multiple wireless control methods and communication protocols
- **Multi-Platform Compatibility**: Compatible with ROS2 and various host computers, supporting both wireless and wired communication
- **AI Support**: Compatible with the LeRobot AI project, supporting deep learning, imitation learning, and reinforcement learning
- **Open Source**: Control code and communication interface documentation are open source for secondary development

## Technical Specifications

- **DOF**: 5+1 (5 arm joints + gripper)
- **Payload**: 0.2kg at 0.5m reach
- **Working Range**: Up to 1m diameter workspace
- **Base Rotation**: 360° omnidirectional
- **Control Board**: General Driver Board for Robots with ESP32 MCU
- **Power Requirements**: 7-12.6V (recommended 12V 5A power supply or 3S lithium battery)
- **Servos**: 
  - Pro version: Metal shell ST3235 servos for all joints except gripper
  - Standard version: Engineering plastic shell ST3215 servos
- **Communication**: WiFi, Serial, I2C, UART

## Bridge Architecture

The RoArm-M3 Isaac Sim Bridge consists of several key components:

1. **Device Discovery**: Automatically detects and identifies RoArm-M3 devices
2. **Data Processor**: Processes telemetry data from the devices
3. **Camera Capture**: Integrates with camera feeds for synchronized data capture
4. **Isaac Integration**: Interfaces with NVIDIA Isaac Sim
5. **Mock Device**: Simulates RoArm-M3 devices for testing purposes

## Repository Structure

```
roarm_isaac_bridge/
├── README.md                  # This overview document
├── config.py                  # Configuration settings
├── main.py                    # Main application entry point
├── isaac_extension.py         # Isaac Sim extension
├── src/                       # Source code
│   ├── device_discovery.py    # Device discovery module
│   ├── data_processor.py      # Data processing module
│   ├── camera_capture.py      # Camera capture module
│   ├── isaac_integration.py   # Isaac Sim integration module
│   └── mock_device.py         # Mock device for testing
└── test_integration.py        # Integration tests
```

## Getting Started

1. Connect the RoArm-M3 to your computer via USB
2. Install the required dependencies:
   ```
   pip install pyserial numpy omni-isaac-sim
   ```
3. Run the main application:
   ```
   python main.py
   ```

## Further Documentation

For more detailed information on specific components, control methods, and advanced usage, please refer to the documentation in the respective folders of this repository.

## Support

For technical support or feedback, please visit the [Waveshare Support Page](https://service.waveshare.com/).
