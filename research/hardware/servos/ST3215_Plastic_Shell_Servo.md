# ST3215 Plastic Shell Servo

## Overview

The ST3215 Plastic Shell Servo is a high-torque programmable serial bus servo used in the RoArm-M3 robotic arm, specifically for the gripper joint in the Pro version. It features a 360° high-precision magnetic encoder, which enables absolute angle control with high precision. The servo is housed in a plastic shell, making it lighter than its metal-shell counterpart (ST3235) while still providing high torque and precision for robotic applications.

![ST3215 Servo](https://www.waveshare.com/w/upload/5/5f/ST3215_Servo.png)

## Key Features

- **High Torque**: Up to 30kg.cm@12V
- **High Precision**: 360°/4096 resolution (0.088° precision)
- **Programmable**: Multiple operation modes (servo angle control/continuous rotation)
- **Feedback**: Real-time position, load, speed, voltage, current, and temperature feedback
- **Daisy-Chaining**: Up to 253 servos can be connected in series
- **Acceleration Control**: Built-in acceleration start-stop function for smoother movement
- **Wide Voltage Range**: 6-12.6V input (compatible with 2S or 3S lithium batteries)
- **No Mechanical Limit**: Can rotate a full 360° in servo mode

## Technical Specifications

### General Specifications
- **Input Voltage**: 6-12.6V
- **Mechanical Limited Angle**: No Limit
- **Rotating Angle**: 360° (servo mode angle control) / motor mode continuous rotation
- **Baudrate**: 1Mbps
- **Gear**: High-precision metal gear
- **Idling Speed**: 0.222sec/60° (45RPM) @12V
- **Position Sensor Resolution**: 360°/4096
- **ID Range**: 0-253
- **Feedback Parameters**: Position, Load, Speed, Input Voltage
- **Idling Current**: 180 mA
- **Locked-rotor Current**: 2.7 A
- **Dimensions**: 45.22mm × 35mm × 24.72mm

## Role in RoArm-M3 Pro

In the RoArm-M3 Pro robotic arm, the ST3215 Plastic Shell Servo is used specifically in the gripper joint. While the other joints use the metal shell ST3235 servos, the gripper uses the ST3215 due to:

1. **Weight Considerations**: The plastic shell makes it lighter, which is beneficial for the end effector
2. **Sufficient Strength**: The gripper requires less torque than the main arm joints
3. **Cost Efficiency**: Plastic shell servos are more cost-effective where metal shells aren't necessary

## Programming and Control

The ST3215 servo can be controlled through several methods:

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

The ST3215 servo supports various commands for control and feedback:

1. **ID Management**: Change servo ID (0-253)
2. **Position Control**: Set absolute position (0-4095)
3. **Speed Control**: Set rotation speed
4. **Torque Control**: Enable/disable torque
5. **Mode Setting**: Switch between servo mode and motor mode
6. **Feedback Reading**: Read position, speed, load, voltage, current, and temperature

## Available Resources

### Documentation
- [ST3215 Servo Wiki Page](https://www.waveshare.com/wiki/ST3215_Servo)
- [Bus Servo Drive Circuit Schematic](https://www.waveshare.com/w/upload/d/d3/Bus_servo_drive_circuit_schematic.pdf)
- [User Manual](https://files.waveshare.com/upload/f/f4/ST3215_Servo_User_Manual.pdf)
- [Communication Protocol User Manual](https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual.pdf)
- [Bus Servo Communication Protocol](https://files.waveshare.com/wiki/common/Smart%20Bus%20Servo%20Communication%20Protocol.pdf)
- [Memory Table](https://files.waveshare.com/upload/2/27/ST3215%20memory%20table.pdf)

### Software
- [ServoDriverST Open Source Program (Arduino)](https://files.waveshare.com/upload/d/d8/ST_Servo.zip)
- [ESP32 Flash Download Tool](https://files.waveshare.com/upload/8/8b/SERVO_DOWNLOAD_TOOL.zip)
- [Arduino IDE](https://www.arduino.cc/en/Main/Software)

### 3D Models
- [ST3215 Servo STEP Module](https://files.waveshare.com/upload/5/59/ST3215-3D.zip)
- [12DOF Robotic Dog STEP Model](https://files.waveshare.com/upload/b/b5/ROBOTIC_DOG_STEP.zip) (Example application)
- [12DOF Robotic Dog Fusion360 Project](https://files.waveshare.com/upload/1/1d/ROBOTIC_DOG_F360.zip) (Example application)
- [12DOF Robotic Dog Creo5.0 Project](https://files.waveshare.com/upload/1/1a/ROBOTIC_DOG_CREO.zip) (Example application)

### 2D Drawings
- [2D Drawing](https://files.waveshare.com/upload/0/08/ST3215-2D.zip)

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

### Changing Servo ID
```cpp
#include <SCServo.h>

SMS_STS st;
int ID_ChangeFrom = 1; // Original servo ID (factory default is 1)
int ID_Changeto = 2;   // New ID

void setup() {
  Serial1.begin(1000000);
  st.pSerial = &Serial1;
  while(!Serial1) {}

  st.unLockEprom(ID_ChangeFrom); // Unlock EPROM-SAFE
  st.writeByte(ID_ChangeFrom, SMS_STS_ID, ID_Changeto); // Change ID
  st.LockEprom(ID_Changeto); // EPROM-SAFE is locked
}

void loop() {
  // Nothing to do here
}
```

## Leader-Follower Mode Implementation

The ST3215 servo can be used in a leader-follower configuration using the ESP32-based Servo Driver. This allows one servo to mimic the movements of another servo in real-time. The implementation uses the ESP-NOW protocol for wireless communication between the leader and follower boards.

### Setup Process:
1. Connect the leader servo to one Servo Driver board
2. Connect the follower servo to another Servo Driver board
3. Upload the ServoDriverST program to both boards
4. Configure the leader board by pressing the "Leader" button in the web interface
5. Note the MAC address of the follower board
6. Update the `broadcastAddress[]` array in the ServoDriverST.ino file with the follower's MAC address
7. Upload the updated program to the leader board
8. Configure the follower board by pressing the "Follower" button in the web interface

The leader board will continuously send the ID, position, and speed of the current Active ID servo to the follower board, which will then replicate these movements.

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

The ST3215 Plastic Shell Servo is a versatile and high-performance servo that provides precise control and real-time feedback for the gripper joint of the RoArm-M3 Pro robotic arm. Its programmable nature, high precision, and ability to be daisy-chained with other servos make it ideal for complex robotic applications.
