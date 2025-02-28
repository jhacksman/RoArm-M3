# RoArm-M3 Pro Documentation

## Overview

The RoArm-M3 Pro is a 5+1 DOF (Degrees of Freedom) smart robotic arm designed for innovative applications. It features a lightweight structure with an effective load capacity of 0.2kg when grabbing objects 0.5m away from the mechanical arm's origin. The Pro version differs from the standard M3 model by using metal shell servos (ST3235) for all joints except the gripper, providing reduced backlash that doesn't increase over time.

![RoArm-M3](https://www.waveshare.com/w/upload/8/8f/360px-RoArm-M3-S-2.jpg)

## Key Features

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

## Control Methods

The RoArm-M3 Pro can be controlled through multiple interfaces:

1. **Web Interface**: Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678) and access the web control panel at 192.168.4.1
2. **JSON Commands**: Send structured commands via:
   - Web interface input box
   - HTTP requests
   - Serial communication
3. **Python API**: Control using provided Python libraries via HTTP or Serial communication
4. **ROS2 Integration**: Compatible with Robot Operating System 2

## Hardware Components

The RoArm-M3 Pro consists of several key components:

1. **Robotic Arm Structure**: 5+1 DOF mechanical arm with metal shell servos
2. **General Driver Board for Robots**: Main control board with ESP32 MCU
3. **Servo Motors**: ST3235 metal shell servos (Pro version)
4. **End Effector**: Gripper with adjustable opening angle
5. **OLED Display**: Shows WiFi status and device information

## Available Resources

- [Open Source Program for Lower Computer](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3-S_Arduino_Demo.zip)
- [Python Demo](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3-S_Python_Demo.zip)
- [3D Model Files](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3-S_3D_Mode.zip)
- [2D Dimension Diagrams](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3-S_2D_Dimensions.zip)
- [Driver Board Schematic](https://files.waveshare.com/wiki/common/General_Driver_for_Robots_SCH.pdf)
- [Initialization Download Tool](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3-S_Init_Tool.zip)

## Repository Structure

This repository contains comprehensive documentation and resources for the RoArm-M3 Pro:

```
roarm-research/
├── README.md                  # This overview document
├── hardware_components/       # Documentation for hardware components
├── software/                  # Software resources and guides
├── drivers/                   # Driver files and installation guides
├── 3d_models/                 # 3D model files for the arm
├── tutorials/                 # Usage tutorials and guides
├── videos/                    # Video resources
├── datasheets/                # Component datasheets
├── leader_follower_mode/      # Documentation for leader-follower functionality
├── images/                    # Images and diagrams
└── research/                  # Detailed research documents for components
```

## Safety Precautions

1. The operating voltage range is 7-12.6V. It is recommended to use the standard 12V 5A power supply or 3S lithium battery.
2. Do not use power supplies that exceed the operating voltage range.
3. The servos have high torque, which may pose potential risks. Avoid having sensitive areas such as eyes or head within the range of the servo's movement.
4. Keep away from children to avoid injury.
5. Avoid subjecting the arm to severe impacts.

## Getting Started

1. Connect the 12V 5A power cable to the power interface on the robotic arm
2. Turn on the power switch - the device will start up with all joints automatically moving to the middle position
3. Connect to the WiFi hotspot named "RoArm-M3" (password: 12345678)
4. Open a web browser and navigate to 192.168.4.1 to access the web control interface

## Further Documentation

For more detailed information on specific components, control methods, and advanced usage, please refer to the documentation in the respective folders of this repository.

## Support

For technical support or feedback, please visit the [Waveshare Support Page](https://service.waveshare.com/).
