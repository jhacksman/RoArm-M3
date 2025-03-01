# ESP32 Firmware for RoArm-M3 Pro

This directory contains the ESP32 firmware for the RoArm-M3 Pro robotic arm, with a focus on the Leader-Follower mode functionality.

## Firmware Overview

The firmware is based on the Arduino framework and provides the following functionality:

- Core control of the RoArm-M3 Pro servos and sensors
- WiFi connectivity for remote control
- Web interface for configuration and control
- ESP-NOW protocol for Leader-Follower mode
- JSON command system for programmatic control

## Key Components

The firmware consists of several modules:

- **Main Control**: Core functionality for controlling the arm
- **ESP-NOW Control**: Implementation of the ESP-NOW protocol for Leader-Follower mode
- **WiFi Control**: WiFi connectivity and configuration
- **JSON Command System**: Processing of JSON commands
- **Web Interface**: Web-based control interface

## ESP-NOW Implementation

The ESP-NOW functionality is implemented in the `esp_now_ctrl.h` and `esp_now_ctrl.cpp` files. These files contain the code for:

- Initializing the ESP-NOW protocol
- Setting up Leader and Follower modes
- Managing peer connections
- Sending and receiving commands between arms

## Installation

For detailed installation instructions, refer to the [main firmware README](../README.md).

## Customization

If you need to customize the firmware for your specific needs:

1. Modify the appropriate files in the firmware project
2. Recompile and upload the firmware using Arduino IDE or PlatformIO
3. Test your changes to ensure they work as expected

## Security Considerations

The ESP-NOW protocol used by the RoArm-M3 Pro relies on MAC addresses for device identification, which presents some security considerations:

1. **MAC Address Authentication**: The system uses MAC addresses for authentication, which can potentially be spoofed
2. **No Encryption**: By default, ESP-NOW does not encrypt the data transmitted between devices
3. **Physical Access Control**: Ensure that only authorized personnel have physical access to the arms
4. **Isolated Network**: Consider operating the arms on an isolated network to prevent unauthorized access

For enhanced security:
- Regularly update the firmware to the latest version
- Use the arms in a controlled environment
- Consider implementing additional authentication mechanisms if using in sensitive applications
