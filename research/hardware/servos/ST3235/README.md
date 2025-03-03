# ST3235 Metal Shell Servo Resources

This directory contains local copies of resources for the ST3235 Metal Shell Servo used in the RoArm-M3 Pro.

## Specifications

- **Dimensions**: 45.22 × 35 × 24.72 mm
- **Weight**: 158g
- **Torque**: 30 kg·cm (2.94 N.m) at 12V
- **Speed**: 0.222s/60° (45 RPM) at 12V
- **Operating Voltage**: 6-12.6V
- **Resolution**: 4096 steps (0.088°)
- **Rotation**: 360° continuous rotation
- **Feedback**: Position, Load, Speed, Input Voltage, Current, Temperature
- **Case Material**: CNC Aluminum
- **Gears**: All-metal
- **Bearings**: Dual ball bearings
- **Communication**: Serial (TTL)
- **Gear Ratio**: 1:345

## Contents

### Drivers

- `ST_Servo.zip`: Driver software and utilities for ST series servos
- `SERVO_DOWNLOAD_TOOL.zip`: Tool for updating servo firmware

### Datasheets

- `ST3235_Datasheet.pdf`: Technical specifications and documentation for the ST3235 servo
- `ST3235_Memory_Table.pdf`: Memory map and register descriptions
- `Smart_Bus_Servo_Communication_Protocol.pdf`: Communication protocol documentation

### 3D Models

- `ST3235-3D.zip`: 3D models of the ST3235 servo in various formats
- `ST3235-2D.zip`: 2D drawings of the ST3235 servo

## Usage

1. Install the appropriate driver software for your operating system
2. Connect the ST3235 servo to your control board
3. Use the provided software to configure and control the servo

## Comparison with Similar Servos

The ST3235 is most comparable to the Dynamixel MX-28 in terms of torque (30 kg·cm vs 25.5 kg·cm), but at a significantly lower price point ($49.99 vs $259.90). Both servos offer 4096-step resolution and comprehensive feedback capabilities.

## Notes

These files are local copies of resources originally provided by Waveshare. They are stored here to ensure availability even if the original sources become inaccessible.

## Original Sources

- [Waveshare ST3235 Product Page](https://www.waveshare.com/st3235-servo.htm)
- [Waveshare ST3235 Wiki](https://www.waveshare.com/wiki/ST3235_Servo)
- [ST3235 Servo User Manual](https://files.waveshare.com/wiki/ST3235/ST3235_Servo_User_Manual.pdf)

## Download Date

These resources were downloaded on March 3, 2025.
