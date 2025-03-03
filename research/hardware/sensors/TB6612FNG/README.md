# TB6612FNG Motor Control Chip Resources

This directory contains local copies of resources for the TB6612FNG Motor Control Chip used in the RoArm-M3 Pro.

## Specifications

- **Manufacturer**: Toshiba
- **Type**: Dual H-bridge motor driver
- **Model**: TB6612FNG
- **Operating Voltage**: 
  - Logic: 2.7V to 5.5V
  - Motor: 2.5V to 13.5V
- **Maximum Output Current**: 1.2A (peak) per channel
- **Continuous Output Current**: 1.0A per channel
- **Standby Control**: Yes (low power consumption mode)
- **PWM Support**: Up to 100kHz
- **Control Modes**: CW, CCW, short brake, stop
- **Package**: SSOP24
- **Operating Temperature**: -40°C to +85°C

## Contents

### Datasheets

- `TB6612FNG_Datasheet.pdf`: Technical specifications and documentation for the TB6612FNG motor driver

### Drivers

- `TB6612FNG_Motor_Control_Firmware.txt`: Information about firmware for the TB6612FNG motor driver
- `TB6612FNG_Example_Code.txt`: Example code for controlling motors with the TB6612FNG

## Usage in RoArm-M3 Pro

The TB6612FNG chip in the RoArm-M3 Pro is used for:

1. Controlling DC motors
2. Providing variable speed control via PWM
3. Enabling direction control (forward/reverse)
4. Implementing braking functionality
5. Power management through standby mode

## Integration with ESP32

The TB6612FNG communicates with the ESP32 main controller via digital I/O pins:

1. The ESP32 provides PWM signals to control motor speed
2. Direction control is managed through digital output pins
3. Standby mode can be toggled to save power when motors are not in use

## Pin Configuration

Typical pin connections between the ESP32 and TB6612FNG:

| TB6612FNG Pin | Function | ESP32 Pin |
|---------------|----------|-----------|
| VM | Motor supply voltage | External power supply |
| VCC | Logic supply voltage | 3.3V |
| GND | Ground | GND |
| STBY | Standby control | GPIO pin |
| AIN1 | Motor A input 1 | GPIO pin |
| AIN2 | Motor A input 2 | GPIO pin |
| PWMA | Motor A PWM input | GPIO PWM pin |
| BIN1 | Motor B input 1 | GPIO pin |
| BIN2 | Motor B input 2 | GPIO pin |
| PWMB | Motor B PWM input | GPIO PWM pin |

## Control Truth Table

| IN1 | IN2 | PWM | STBY | Mode |
|-----|-----|-----|------|------|
| H | L | H | H | CW (Forward) |
| L | H | H | H | CCW (Reverse) |
| L | L | H | H | Short Brake |
| H | H | H | H | Short Brake |
| x | x | L | H | Stop |
| x | x | x | L | Standby |

## Notes

These files are local copies of resources originally provided by Toshiba and Waveshare. They are stored here to ensure availability even if the original sources become inaccessible.

## Original Sources

- [Toshiba TB6612FNG Product Page](https://toshiba.semicon-storage.com/ap-en/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html)
- [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3)

## Download Date

These resources were downloaded on March 3, 2025.
