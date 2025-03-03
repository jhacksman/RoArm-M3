# AngleCtrl: Servo Angle Control for RoArm-M3 Pro

## Overview

The AngleCtrl (Servo Angle Control) is a fundamental control feature of the RoArm-M3 Pro robotic arm that enables precise control of each servo joint's angular position. This control system allows users to manipulate the robotic arm with high precision, controlling each joint independently to achieve desired arm positions and movements. The AngleCtrl feature is accessible through the web interface, JSON commands, and programmatic interfaces, making it versatile for various applications from manual control to automated sequences.

![AngleCtrl Web Interface](https://www.waveshare.com/w/upload/9/95/500px-RAM3-web1%282%29.png)

## Key Features

- **Individual Joint Control**: Independent control of each of the 6 servo joints
- **Precise Angular Positioning**: Control joints with precision measured in radians
- **Multiple Control Interfaces**: Accessible via web interface, JSON commands, and programmatic APIs
- **Real-time Feedback**: Provides current angle values for each joint
- **Automatic Initialization**: Automatically moves all joints to middle positions on startup
- **Safety Limits**: Built-in rotation limits to prevent mechanical damage
- **Speed Control**: Adjustable movement speed for each joint

## Joint Configuration

The RoArm-M3 Pro features 6 controllable joints, each with specific rotation ranges and control parameters:

### 1. Base Joint (B)
- **Control Labels**: "B L" (Base Left) and "B R" (Base Right)
- **Rotation Range**: 360° (3.14 to -3.14 radians)
- **Default Position**: 0 radians (middle position)
- **Function**: Controls the horizontal rotation of the entire arm

### 2. Shoulder Joint (S)
- **Control Labels**: "S D" (Shoulder Down) and "S U" (Shoulder Up)
- **Rotation Range**: 180° (1.57 to -1.57 radians)
- **Default Position**: 0 radians (middle position)
- **Function**: Controls the primary up/down movement of the arm
- **Special Feature**: Uses dual-drive technology for increased torque

### 3. Elbow Joint (E)
- **Control Labels**: "E D" (Elbow Down) and "E U" (Elbow Up)
- **Rotation Range**: 180° (rotation values vary based on position)
- **Default Position**: 1.57 radians (middle position)
- **Function**: Controls the secondary up/down movement of the arm

### 4. Wrist Joint 1 (W)
- **Control Labels**: "W+DG" (Wrist Down/Grip) and "W-UG" (Wrist Up/Grip)
- **Rotation Range**: 180° (1.57 to -1.57 radians)
- **Default Position**: 0 radians (middle position)
- **Function**: Controls the up/down tilt of the wrist

### 5. Wrist Joint 2 (R)
- **Control Labels**: "R+DG" (Rotation Plus) and "R-UG" (Rotation Minus)
- **Rotation Range**: 360° (3.14 to -3.14 radians)
- **Default Position**: 0 radians (middle position)
- **Function**: Controls the rotational movement of the wrist

### 6. End Joint/Gripper (G)
- **Control Labels**: "G+DG" (Grip Close) and "G-UG" (Grip Open)
- **Rotation Range**: 135° (3.14 to 1.08 radians)
- **Default Position**: 3.14 radians (closed position)
- **Function**: Controls the opening and closing of the gripper

## Technical Implementation

### Servo Hardware

The AngleCtrl system interfaces with two types of servo motors in the RoArm-M3 Pro:

1. **ST3235 Metal Shell Servos**: Used in all joints except the gripper in the Pro version
   - 12-bit high-precision magnetic encoder (0.088° resolution)
   - Metal gears for reduced backlash
   - Higher torque capacity
   - TTL bus communication protocol

2. **ST3215 Plastic Shell Servo**: Used in the gripper joint and in all joints of the standard version
   - 12-bit magnetic encoder
   - Plastic gears
   - TTL bus communication protocol

### Control Architecture

The AngleCtrl system operates through a hierarchical control architecture:

1. **User Interface Layer**: Web interface, Python API, or custom applications
2. **Command Processing Layer**: ESP32 microcontroller processes commands
3. **Communication Layer**: TTL bus protocol for servo communication
4. **Servo Control Layer**: Individual servo controllers manage precise positioning

### Angle Measurement System

- **Unit of Measurement**: Radians (displayed on the web interface)
- **Precision**: 0.088° (0.0015 radians) with 12-bit encoder
- **Feedback Mechanism**: Real-time angle reporting from servo encoders
- **Coordinate System**: Each joint has its own reference frame and zero position

## Usage Methods

### 1. Web Interface Control

The web interface provides intuitive control of each joint through labeled buttons:

1. Connect to the RoArm-M3 Pro's WiFi (SSID: RoArm-M3, Password: 12345678)
2. Navigate to 192.168.4.1 in a web browser
3. Use the AngleCtrl section to control individual joints:
   - Click and hold buttons to move joints
   - Release to stop movement
   - Current angle values are displayed on each button
   - Click "INIT" to return all joints to their default positions

### 2. JSON Command Control

Direct control via JSON commands allows for programmatic and precise control:

```json
{
  "type": "AngleCtrl",
  "id": 1,
  "angle": 1.57,
  "speed": 50
}
```

Parameters:
- `type`: Command type (AngleCtrl)
- `id`: Joint ID (1-6, corresponding to the 6 joints)
- `angle`: Target angle in radians
- `speed`: Movement speed (0-100, where 0 is maximum speed)

### 3. Python API Control

Control through Python scripts using either HTTP or Serial communication:

#### HTTP Example:
```python
import requests
import json

def angle_control(ip_address, joint_id, angle, speed):
    cmd = {
        "type": "AngleCtrl",
        "id": joint_id,
        "angle": angle,
        "speed": speed
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    return response.text

# Control base joint to 90 degrees (1.57 radians) at 50% speed
angle_control("192.168.4.1", 1, 1.57, 50)
```

#### Serial Example:
```python
import serial
import json

def angle_control(port, joint_id, angle, speed):
    cmd = {
        "type": "AngleCtrl",
        "id": joint_id,
        "angle": angle,
        "speed": speed
    }
    with serial.Serial(port, baudrate=115200) as ser:
        ser.write(json.dumps(cmd).encode() + b'\n')

# Control shoulder joint to 45 degrees (0.785 radians) at 30% speed
angle_control("/dev/ttyUSB0", 2, 0.785, 30)
```

## Joint ID Reference

For programmatic control, use the following joint ID mapping:

| Joint ID | Joint Name | Control Labels |
|----------|------------|----------------|
| 1 | Base Joint | B L / B R |
| 2 | Shoulder Joint | S D / S U |
| 3 | Elbow Joint | E D / E U |
| 4 | Wrist Joint 1 | W+DG / W-UG |
| 5 | Wrist Joint 2 | R+DG / R-UG |
| 6 | End Joint/Gripper | G+DG / G-UG |

## Common Use Cases

### 1. Initialization Sequence

Return all joints to their default positions:

```json
{"type":"AngleCtrl","id":1,"angle":0,"speed":50}
{"type":"AngleCtrl","id":2,"angle":0,"speed":50}
{"type":"AngleCtrl","id":3,"angle":1.57,"speed":50}
{"type":"AngleCtrl","id":4,"angle":0,"speed":50}
{"type":"AngleCtrl","id":5,"angle":0,"speed":50}
{"type":"AngleCtrl","id":6,"angle":3.14,"speed":50}
```

### 2. Pick and Place Sequence

A basic pick and place operation might involve:

1. Move to approach position
2. Open gripper
3. Move to pick position
4. Close gripper
5. Move to place position
6. Open gripper
7. Return to home position

### 3. Coordinated Joint Movement

For smooth, natural-looking movements, coordinate multiple joints:

```python
# Example of coordinated movement in Python
def coordinated_movement(ip_address, positions, speed):
    for joint_id, angle in positions.items():
        angle_control(ip_address, joint_id, angle, speed)
        time.sleep(0.1)  # Small delay between commands for smoother motion

# Move arm to a specific pose
coordinated_movement("192.168.4.1", {
    1: 0.5,    # Base
    2: -0.3,   # Shoulder
    3: 1.2,    # Elbow
    4: 0.7,    # Wrist 1
    5: 1.5,    # Wrist 2
    6: 2.5     # Gripper
}, 40)
```

## Integration with Other Control Features

The AngleCtrl system integrates with other RoArm-M3 Pro control features:

### 1. Torque Lock Control

- When Torque Lock is disabled, joints can be manually positioned
- AngleCtrl commands automatically re-enable Torque Lock

### 2. DEFA (Dynamic External Force Adaptive Control)

- When DEFA is enabled, the arm resists external forces
- AngleCtrl commands take precedence over DEFA

### 3. COORDCTRL (End Point Coordinate Control)

- COORDCTRL uses inverse kinematics to calculate joint angles
- AngleCtrl provides direct joint control as an alternative

## Performance Considerations

### 1. Speed and Acceleration

- Higher speed values result in slower, more controlled movements
- Speed value of 0 uses maximum speed
- Consider using moderate speeds (40-60) for precise positioning

### 2. Load Capacity

- Joint performance varies based on load and position
- The shoulder joint (with dual-drive technology) has the highest torque capacity
- Consider load distribution when planning movements

### 3. Precision and Backlash

- Metal shell servos (ST3235) provide better precision and less backlash
- Plastic shell servos (ST3215) may develop increased backlash over time
- For highest precision applications, use slower speeds and consider position verification

## Troubleshooting

### Common Issues

1. **Jerky Movement**
   - **Symptom**: Joints move in an uneven or jerky manner
   - **Possible Causes**: Speed set too low, mechanical resistance, servo issues
   - **Solutions**: Increase speed value, check for mechanical obstructions, verify power supply

2. **Position Drift**
   - **Symptom**: Joints don't maintain their position under load
   - **Possible Causes**: Torque limit reached, servo overheating
   - **Solutions**: Reduce load, check power supply, allow servos to cool

3. **Unresponsive Joints**
   - **Symptom**: Joints don't respond to commands
   - **Possible Causes**: Communication issues, servo failure
   - **Solutions**: Check connections, verify command format, restart system

4. **Oscillation**
   - **Symptom**: Joint oscillates around target position
   - **Possible Causes**: Speed too high, mechanical issues
   - **Solutions**: Reduce speed, check for mechanical play

## Available Resources

### Documentation
- [RoArm-M3 Wiki Page](https://www.waveshare.com/wiki/RoArm-M3)
- [JSON Command System Documentation](./JSON_Command_System.md)
- [Python API Documentation](./Python_API.md)

### Hardware Resources
- [ST3235 Metal Shell Servo Datasheet](https://www.waveshare.com/st3235-servo.htm)
- [ST3215 Plastic Shell Servo Datasheet](https://www.waveshare.com/st3215-servo.htm)
- [General Driver Board for Robots Schematic](https://files.waveshare.com/wiki/common/General_Driver_for_Robots_SCH.pdf) ([Local Copy](../hardware/main_control/ESP32-WROOM-32/datasheets/General_Driver_for_Robots_SCH.pdf))

### Software Resources
- [RoArm-M3 Python Demo](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip) ([Local Copy](../hardware/misc/misc/drivers/RoArm-M3_Python.zip))
- [RoArm-M3 Open Source Program](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Open_Source_Program.zip) ([Local Copy](../hardware/misc/misc/drivers/RoArm-M3_Open_Source_Program.zip))

## Conclusion

The AngleCtrl (Servo Angle Control) system is the foundation of the RoArm-M3 Pro's movement capabilities, providing precise control over each joint. By understanding the joint configuration, control methods, and integration with other features, users can fully leverage the arm's capabilities for a wide range of applications from simple positioning tasks to complex automated sequences. Whether controlled through the web interface, JSON commands, or programmatic APIs, the AngleCtrl system offers the flexibility and precision needed for advanced robotic applications.
