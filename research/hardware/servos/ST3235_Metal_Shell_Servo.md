# ST3235 Metal Shell Servo

## Overview

The ST3235 Metal Shell Servo is a high-torque programmable serial bus servo used in the RoArm-M3 Pro robotic arm. It features an aluminum alloy case and is equipped with a 360° high-precision magnetic encoder, which enables absolute angle control with high precision. This servo is a key differentiator of the Pro version of the RoArm-M3, providing reduced backlash that doesn't increase over time compared to the plastic shell servos used in the standard version.

![ST3235 Servo](https://www.waveshare.com/w/upload/thumb/2/22/ST3235_Servo.jpg/300px-ST3235_Servo.jpg)

## Key Features

- **High Torque**: Up to 30kg.cm@12V
- **High Precision**: 360°/4096 resolution (0.088° precision)
- **Metal Shell**: Aluminum alloy case with CNC processing
- **Programmable**: Multiple operation modes (servo angle control/continuous rotation)
- **Feedback**: Real-time position, load, speed, voltage, current, and temperature feedback
- **Daisy-Chaining**: Up to 253 servos can be connected in series
- **Acceleration Control**: Built-in acceleration start-stop function for smoother movement
- **Wide Voltage Range**: 6-12.6V input (compatible with 2S or 3S lithium batteries)
- **No Mechanical Limit**: Can rotate a full 360° in servo mode

## Technical Specifications

### General Specifications
- **Input Voltage**: 6-12.6V
- **Mechanism Limit Angle**: No Limit
- **Rotating Angle**: 360° (0~4095 in servo mode) / continuous rotation in motor mode
- **Baudrate**: 1Mbps
- **Gear**: High precision steel gear (reduction ratio 1:345)
- **Idling Speed**: 0.222sec/60° (45RPM@12V)
- **Position Sensor Resolution**: 360°/4096
- **ID Range**: 1 ~ 253 (1 by default)
- **Feedback Parameters**: Position, Load, Speed, Input Voltage, Current, Temperature
- **Idling Current**: 190mA
- **Locked-rotor Current**: 2.7A
- **Dimensions**: 45.22mm × 35mm × 24.72mm

## Role in RoArm-M3 Pro

In the RoArm-M3 Pro robotic arm, the ST3235 Metal Shell Servo is used in all joints except the gripper. The metal shell design provides several advantages over the plastic shell servos used in the standard version:

1. **Reduced Backlash**: The metal construction minimizes gear play
2. **Consistent Performance**: Backlash doesn't increase over time
3. **Higher Durability**: More resistant to wear and tear
4. **Better Heat Dissipation**: Metal shell helps dissipate heat during operation
5. **Higher Precision**: Contributes to the overall precision of the robotic arm

## Programming and Control

The ST3235 servo can be controlled through several methods:

### Hardware Connection

The servo uses a TTL Serial Bus interface and can be controlled using:

1. **Servo Driver Board**: Waveshare provides a dedicated [Servo Driver with ESP32](https://www.waveshare.com/servo-driver-with-esp32.htm)
2. **Custom Circuit**: A custom circuit can be built following the [Bus servo drive circuit schematic](https://www.waveshare.com/w/upload/d/d3/Bus_servo_drive_circuit_schematic.pdf)

### Software Control

The servo can be programmed using:

1. **ESP32 Flash Download Tool**: A simplified method without requiring Arduino IDE
2. **Arduino IDE**: Using the provided libraries and demo code
3. **Custom Software**: Using the serial communication protocol

### Control Commands

The ST3235 servo supports various commands for control and feedback:

1. **ID Management**: Change servo ID (1-253)
2. **Position Control**: Set absolute position (0-4095)
3. **Speed Control**: Set rotation speed
4. **Torque Control**: Enable/disable torque
5. **Mode Setting**: Switch between servo mode and motor mode
6. **Feedback Reading**: Read position, speed, load, voltage, current, and temperature

## Available Resources

### Documentation
- [ST3235 Servo Wiki Page](https://www.waveshare.com/wiki/ST3235_Servo)
- [Bus Servo Control Circuit Schematic](https://www.waveshare.com/w/upload/d/d3/Bus_servo_drive_circuit_schematic.pdf)
- [User Manual](https://files.waveshare.com/upload/b/b5/ST3215_Servo_User_Manual.pdf) ([Local Copy](ST3215/datasheets/ST3215_Servo_User_Manual.pdf))

### Software
- [ServoDriverST Open Source Program (Arduino)](https://files.waveshare.com/upload/d/d8/ST_Servo.zip) ([Local Copy](ST3235/drivers/ST_Servo.zip))
- [ESP32 Flash Download Tool](https://files.waveshare.com/upload/8/8b/SERVO_DOWNLOAD_TOOL.zip) ([Local Copy](ST3235/drivers/SERVO_DOWNLOAD_TOOL.zip))
- [Arduino IDE](https://www.arduino.cc/en/Main/Software)

### 3D Models
- [ST3215/ST3235 Servo STEP Module](https://files.waveshare.com/upload/b/b5/ST3215-3D.zip) ([Local Copy](ST3235/drivers/ST3215-3D.zip))
- [12DOF Robotic Dog STEP Model](https://files.waveshare.com/upload/b/b5/ROBOTIC_DOG_STEP.zip) ([Local Copy](ST3235/drivers/ROBOTIC_DOG_STEP.zip)) (Example application)
- [12DOF Robotic Dog Fusion360 Project](https://files.waveshare.com/upload/1/1d/ROBOTIC_DOG_F360.zip) ([Local Copy](ST3235/drivers/ROBOTIC_DOG_F360.zip)) (Example application)
- [12DOF Robotic Dog Creo5.0 Project](https://files.waveshare.com/upload/1/1a/ROBOTIC_DOG_CREO.zip) ([Local Copy](ST3235/drivers/ROBOTIC_DOG_CREO.zip)) (Example application)

### 2D Drawings
- [2D Drawing](https://files.waveshare.com/upload/b/b5/ST3215-2D.zip) ([Local Copy](ST3235/drivers/ST3215-2D.zip))

## Programming Examples

### Servo Initialization
```cpp
#include <SCServo.h>

SMS_STS st;

void setup() {
  Serial1.begin(1000000); // Initialize the serial port
  st.pSerial = &Serial1;
  while(!Serial1) {}
}
```

### Position Control
```cpp
#include <SCServo.h>

SMS_STS st;

void setup() {
  Serial1.begin(1000000);
  st.pSerial = &Serial1;
  while(!Serial1) {}
}

void loop() {
  st.WritePos(1, 1000, 1500, 50); // Control servo ID 1 to position 1000 at speed 1500 with acceleration 50
  delay(754); // [(P1-P0)/V]*1000+100

  st.WritePos(1, 20, 1500, 50); // Control servo ID 1 to position 20
  delay(754);
}
```

### Synchronous Control of Multiple Servos
```cpp
#include <SCServo.h>

SMS_STS st;
byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];

void setup() {
  Serial1.begin(1000000);
  st.pSerial = &Serial1;
  delay(1000);
  
  ID[0] = 1;   // Servo IDs
  ID[1] = 2;
  Speed[0] = 3400;  // Speeds
  Speed[1] = 3400;
  ACC[0] = 50;   // Accelerations
  ACC[1] = 50;
}

void loop() {
  Position[0] = 3000;  // Target positions
  Position[1] = 3000;
  st.SyncWritePosEx(ID, 2, Position, Speed, ACC);
  delay(2000);

  Position[0] = 100;
  Position[1] = 100;
  st.SyncWritePosEx(ID, 2, Position, Speed, ACC);
  delay(2000);
}
```

### Reading Feedback
```cpp
#include <SCServo.h>

SMS_STS sms_sts;

void setup() {
  Serial1.begin(1000000);
  Serial.begin(115200);
  sms_sts.pSerial = &Serial1;
  delay(1000);
}

void loop() {
  if(sms_sts.FeedBack(1) != -1) {
    int Pos = sms_sts.ReadPos(-1);
    int Speed = sms_sts.ReadSpeed(-1);
    int Load = sms_sts.ReadLoad(-1);
    int Voltage = sms_sts.ReadVoltage(-1);
    int Temper = sms_sts.ReadTemper(-1);
    int Move = sms_sts.ReadMove(-1);
    int Current = sms_sts.ReadCurrent(-1);
    
    Serial.print("Position: ");
    Serial.println(Pos);
    Serial.print("Speed: ");
    Serial.println(Speed);
    Serial.print("Load: ");
    Serial.println(Load);
    Serial.print("Voltage: ");
    Serial.println(Voltage);
    Serial.print("Temperature: ");
    Serial.println(Temper);
    Serial.print("Move: ");
    Serial.println(Move);
    Serial.print("Current: ");
    Serial.println(Current);
    
    delay(1000);
  }
}
```

## Troubleshooting

### Common Issues

1. **Communication Problems**
   - Ensure correct baud rate (1Mbps)
   - Check wiring connections
   - Verify servo ID is correct and unique

2. **Power Issues**
   - Ensure adequate power supply (6-12.6V)
   - Check for sufficient current capacity
   - Use separate power for logic and servo power

3. **Overheating**
   - Monitor temperature feedback
   - Avoid continuous high-load operation
   - Ensure adequate cooling

4. **Position Control Issues**
   - Verify position values are within range (0-4095)
   - Check for mechanical obstructions
   - Ensure torque is enabled

## Conclusion

The ST3235 Metal Shell Servo is a high-performance servo that provides precise control, real-time feedback, and durability for the RoArm-M3 Pro robotic arm. Its metal construction and high-precision encoder make it ideal for applications requiring accurate positioning and consistent performance over time.
