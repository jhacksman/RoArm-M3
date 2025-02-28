# TB6612FNG Motor Control Chip

## Overview

The TB6612FNG is a dual H-bridge motor driver integrated circuit (IC) manufactured by Toshiba, used in the RoArm-M3 Pro robotic arm's General Driver Board for Robots. This chip enables the control of DC motors, providing bidirectional drive capability with output transistors in a low ON-resistance structure. In the RoArm-M3 Pro, it primarily controls the DC motors that may be connected to the robotic arm's driver board.

![TB6612FNG Chip](https://cdn.sparkfun.com/assets/parts/3/1/5/7/09457-01.jpg)

## Key Features

- **Dual H-Bridge Driver**: Can independently control two bidirectional DC motors
- **Wide Voltage Range**: Motor supply voltage (VM) from 2.5V to 13.5V
- **High Current Capability**: 1.2A continuous output current (3.2A peak) per channel
- **Low ON-Resistance**: Reduces power dissipation and improves efficiency
- **Multiple Control Modes**: CW/CCW/short brake/stop functions
- **Standby Control**: Power-saving standby mode
- **Built-in Protection**: Thermal shutdown circuit and undervoltage lockout
- **Small Package Size**: Available in SSOP24 package

## Technical Specifications

### Electrical Characteristics
- **Motor Supply Voltage (VM)**: 2.5V to 13.5V
- **Logic Supply Voltage (VCC)**: 2.7V to 5.5V
- **Output Current**: 1.2A (continuous), 3.2A (peak) per channel
- **Standby Current**: 0μA (typical)
- **Operating Temperature Range**: -20°C to +85°C
- **Internal Pull-Down Resistors**: On inputs to prevent floating
- **PWM Frequency**: Up to 100kHz

### Control Inputs
- **IN1/IN2 (Channel A)**: Logic inputs for motor direction control
- **IN3/IN4 (Channel B)**: Logic inputs for motor direction control
- **PWM**: Pulse-width modulation input for speed control
- **STBY**: Standby control input (active low)

### Truth Table for Motor Control

| IN1 | IN2 | Mode |
|-----|-----|------|
| H   | H   | Short Brake |
| L   | H   | Clockwise Rotation |
| H   | L   | Counter-Clockwise Rotation |
| L   | L   | Stop |

*Note: When STBY = L, the motor is in standby mode regardless of other inputs.*

## Role in RoArm-M3 Pro

In the RoArm-M3 Pro robotic arm, the TB6612FNG chip is integrated into the General Driver Board for Robots and serves several functions:

1. **Motor Control**: Provides the ability to control DC motors that may be connected to the robotic arm for additional functionality
2. **Interface Management**: Manages the motor interfaces (both with and without encoders)
3. **Power Efficiency**: Ensures efficient power usage through low ON-resistance and standby mode
4. **Protection**: Protects motors and circuitry from overcurrent and thermal issues

The General Driver Board features four motor interfaces that utilize the TB6612FNG:
- Two PH2.0 6P interfaces for motors with encoders (Groups A and B)
- Two PH2.0 2P interfaces for motors without encoders (Groups A and B)

## Programming and Integration

The TB6612FNG in the RoArm-M3 Pro can be controlled through the ESP32 microcontroller using GPIO pins. Waveshare provides demo code and libraries for interfacing with the TB6612FNG chip.

### Pin Connections on General Driver Board

| TB6612FNG Pin | ESP32 GPIO | Function |
|---------------|------------|----------|
| PWMA          | GPIO 25    | PWM control for Motor A |
| AIN1          | GPIO 26    | Direction control 1 for Motor A |
| AIN2          | GPIO 27    | Direction control 2 for Motor A |
| PWMB          | GPIO 12    | PWM control for Motor B |
| BIN1          | GPIO 14    | Direction control 1 for Motor B |
| BIN2          | GPIO 13    | Direction control 2 for Motor B |
| STBY          | GPIO 23    | Standby control |

### Basic Control Logic

To control a motor using the TB6612FNG:

1. Set STBY pin HIGH to enable the driver
2. Set direction using IN1/IN2 pins according to the truth table
3. Apply PWM signal to control speed
4. Set STBY pin LOW when motors are not in use to save power

## Available Resources

### Documentation
- [TB6612FNG Datasheet (Toshiba)](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf)
- [General Driver for Robots Wiki Page](https://www.waveshare.com/wiki/General_Driver_for_Robots)
- [Tutorial IV: Motor Without Encoder Control Demo](https://www.waveshare.com/wiki/Tutorial_II:_Motor_Without_Encoder_Control_Demo)
- [Tutorial I: Motor With Encoder Control Demo](https://www.waveshare.com/wiki/Tutorial_I:_Motor_With_Encoder_Control_Demo)

### Software
- [UGV01 Open-source Demo](https://files.waveshare.com/upload/0/0c/UGV01_Basic_Demo.zip) (Contains TB6612FNG control examples)
- [Arduino TB6612FNG Library by SparkFun](https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library)
- [Adafruit TB6612 Library](https://github.com/adafruit/Adafruit_TB6612)

### Hardware Resources
- [Circuit Diagram](https://files.waveshare.com/upload/3/37/General_Driver_for_Robots_SCH.pdf)
- [General Driver for Robots STEP Model](https://files.waveshare.com/upload/8/8e/General_Driver_for_Robots_STEP.zip)

## Programming Examples

### Arduino Example (Basic Motor Control)
```cpp
#include <Arduino.h>

// Motor A
#define PWMA 25
#define AIN1 26
#define AIN2 27

// Motor B
#define PWMB 12
#define BIN1 14
#define BIN2 13

// Standby
#define STBY 23

void setup() {
  // Configure pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Enable the driver
  digitalWrite(STBY, HIGH);
}

void loop() {
  // Motor A forward at half speed
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 128);  // 0-255 for speed control
  
  // Motor B forward at full speed
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);
  
  delay(2000);
  
  // Short brake both motors
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  
  delay(1000);
  
  // Motor A backward at half speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 128);
  
  // Motor B backward at full speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 255);
  
  delay(2000);
  
  // Stop both motors
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  
  delay(1000);
  
  // Enter standby mode to save power
  digitalWrite(STBY, LOW);
  delay(3000);
  digitalWrite(STBY, HIGH);  // Exit standby mode
}
```

### ESP32 Example for RoArm-M3 Pro (Using SparkFun Library)
```cpp
#include <Arduino.h>
#include <SparkFun_TB6612.h>

// Motor A
#define AIN1 26
#define AIN2 27
#define PWMA 25

// Motor B
#define BIN1 14
#define BIN2 13
#define PWMB 12

// Standby
#define STBY 23

// Declare motors
Motor motorA = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motorB = Motor(BIN1, BIN2, PWMB, 1, STBY);

void setup() {
  Serial.begin(115200);
  Serial.println("TB6612FNG Motor Control Example for RoArm-M3 Pro");
}

void loop() {
  // Drive forward
  Serial.println("Driving forward");
  motorA.drive(255);  // Full speed forward
  motorB.drive(255);
  delay(1000);
  
  // Brake
  Serial.println("Braking");
  motorA.brake();
  motorB.brake();
  delay(500);
  
  // Drive backward
  Serial.println("Driving backward");
  motorA.drive(-255);  // Full speed backward
  motorB.drive(-255);
  delay(1000);
  
  // Brake
  motorA.brake();
  motorB.brake();
  delay(500);
  
  // Variable speed control
  Serial.println("Variable speed control");
  for (int i = 0; i <= 255; i++) {
    motorA.drive(i);
    motorB.drive(i);
    delay(10);
  }
  
  for (int i = 255; i >= -255; i--) {
    motorA.drive(i);
    motorB.drive(i);
    delay(10);
  }
  
  for (int i = -255; i <= 0; i++) {
    motorA.drive(i);
    motorB.drive(i);
    delay(10);
  }
  
  // Stop
  Serial.println("Stopping");
  motorA.brake();
  motorB.brake();
  
  // Enter standby mode
  Serial.println("Entering standby mode");
  motorA.standby();  // This will set STBY pin LOW
  delay(2000);
  
  // Exit standby mode is automatic when drive() is called again
}
```

## Troubleshooting

### Common Issues

1. **Motors Not Responding**
   - Check STBY pin is HIGH
   - Verify power supply voltage is within 2.5V-13.5V range
   - Ensure proper connections to ESP32 GPIO pins

2. **Overheating**
   - Check for short circuits
   - Verify motor current is within specifications (1.2A continuous)
   - Ensure adequate cooling for high-current applications

3. **Erratic Motor Behavior**
   - Check for PWM frequency issues
   - Verify logic supply voltage (VCC) is stable
   - Check for noise on control signal lines

4. **Low Motor Power**
   - Verify motor supply voltage is adequate
   - Check for voltage drops under load
   - Ensure proper grounding

## Limitations and Considerations

1. **Current Limitations**:
   - The continuous current is limited to 1.2A per channel
   - For higher current applications, consider using external MOSFETs or a higher-rated driver

2. **Thermal Considerations**:
   - The chip can get hot during operation, especially at high currents
   - Consider adding a heatsink for continuous high-current applications

3. **Power Supply Requirements**:
   - Separate logic and motor power supplies are recommended for noise isolation
   - Use decoupling capacitors close to the chip for stable operation

4. **PWM Frequency**:
   - Very high PWM frequencies may cause heating in the driver
   - Very low PWM frequencies may cause audible motor noise

## Conclusion

The TB6612FNG Motor Control Chip is a versatile and efficient dual H-bridge driver that provides reliable motor control capabilities for the RoArm-M3 Pro robotic arm. Its low ON-resistance, multiple control modes, and built-in protection features make it ideal for controlling DC motors in robotic applications. When used with the ESP32 microcontroller on the General Driver Board for Robots, it enables precise control of motor speed and direction, enhancing the functionality of the robotic arm system.
