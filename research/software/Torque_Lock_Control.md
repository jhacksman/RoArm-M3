# Torque Lock Control for RoArm-M3 Pro

## Overview

Torque Lock Control is a fundamental feature of the RoArm-M3 Pro robotic arm that allows users to enable or disable the torque applied to the servo motors. When torque is enabled (locked), the servos actively maintain their positions and resist external forces. When torque is disabled (unlocked), the servos can be manually positioned without resistance, enabling teaching by demonstration and manual positioning capabilities. This feature is essential for applications requiring physical human-robot interaction, position teaching, and safety operations.

![Torque Lock Control Web Interface](https://www.waveshare.com/w/upload/9/95/500px-RAM3-web1%282%29.png)

## Key Features

- **Dual Control States**: Toggle between torque on (locked) and torque off (unlocked) modes
- **Manual Positioning**: When torque is off, joints can be freely moved by hand
- **Position Retention**: When torque is on, servos actively maintain their positions
- **Automatic Re-engagement**: Torque automatically re-enables when new movement commands are received
- **Individual Joint Control**: Can be applied to specific joints or all joints simultaneously
- **Web Interface Control**: Accessible through intuitive web-based controls
- **JSON Command Control**: Programmable through standardized JSON commands
- **Safety Mechanism**: Provides a way to manually intervene in case of unexpected behavior

## Technical Implementation

### Servo Hardware Integration

The Torque Lock Control feature interfaces directly with the servo motors in the RoArm-M3 Pro:

1. **ST3235 Metal Shell Servos** (Pro version, all joints except gripper):
   - 12-bit high-precision magnetic encoder
   - Metal gears for reduced backlash
   - TTL bus communication protocol
   - Supports torque control commands

2. **ST3215 Plastic Shell Servo** (Standard version and Pro version gripper):
   - 12-bit magnetic encoder
   - Plastic gears
   - TTL bus communication protocol
   - Supports torque control commands

### Control Architecture

The Torque Lock Control system operates through a hierarchical control architecture:

1. **User Interface Layer**: Web interface, Python API, or custom applications
2. **Command Processing Layer**: ESP32 microcontroller processes torque control commands
3. **Communication Layer**: TTL bus protocol for servo communication
4. **Servo Control Layer**: Individual servo controllers manage torque application

### System States

The Torque Lock Control feature has two primary states:

1. **Torque ON (Locked)**:
   - Servos actively maintain their positions
   - External forces are resisted
   - Position feedback is continuously monitored
   - Power consumption is higher due to active holding

2. **Torque OFF (Unlocked)**:
   - Servos do not resist external forces
   - Joints can be manually positioned
   - Position feedback is still available
   - Power consumption is lower due to passive state

## Usage Methods

### 1. Web Interface Control

The web interface provides intuitive control of the torque lock feature:

1. Connect to the RoArm-M3 Pro's WiFi (SSID: RoArm-M3, Password: 12345678)
2. Navigate to 192.168.4.1 in a web browser
3. Use the Torque Lock Control section:
   - Click "Torque OFF" to disable torque and allow manual positioning
   - Click "Torque ON" to enable torque and maintain positions

### 2. JSON Command Control

Direct control via JSON commands allows for programmatic torque control:

```json
{
  "type": "TorqueCtrl",
  "id": 0,
  "enable": false
}
```

Parameters:
- `type`: Command type (TorqueCtrl)
- `id`: Joint ID (0 for all joints, 1-6 for specific joints)
- `enable`: Torque state (true for ON/locked, false for OFF/unlocked)

### 3. Python API Control

Control through Python scripts using either HTTP or Serial communication:

#### HTTP Example:
```python
import requests
import json

def torque_control(ip_address, joint_id, enable):
    cmd = {
        "type": "TorqueCtrl",
        "id": joint_id,
        "enable": enable
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    return response.text

# Disable torque for all joints
torque_control("192.168.4.1", 0, False)

# Enable torque for joint 2 (shoulder)
torque_control("192.168.4.1", 2, True)
```

#### Serial Example:
```python
import serial
import json

def torque_control(port, joint_id, enable):
    cmd = {
        "type": "TorqueCtrl",
        "id": joint_id,
        "enable": enable
    }
    with serial.Serial(port, baudrate=115200) as ser:
        ser.write(json.dumps(cmd).encode() + b'\n')

# Disable torque for all joints
torque_control("/dev/ttyUSB0", 0, False)

# Enable torque for joint 3 (elbow)
torque_control("/dev/ttyUSB0", 3, True)
```

## Common Use Cases

### 1. Teaching by Demonstration

One of the most powerful applications of the Torque Lock Control feature is teaching by demonstration:

1. Disable torque using the "Torque OFF" command
2. Manually move the arm through the desired motion sequence
3. Record joint positions at key points
4. Re-enable torque using the "Torque ON" command
5. Replay the recorded positions programmatically

### 2. Manual Positioning

For precise positioning without programming:

1. Disable torque using the "Torque OFF" command
2. Manually position the arm as needed
3. Re-enable torque using the "Torque ON" command to maintain the position

### 3. Emergency Intervention

In case of unexpected behavior:

1. Disable torque using the "Torque OFF" command
2. Manually move the arm to a safe position
3. Diagnose and resolve the issue
4. Re-enable torque when ready to resume operation

### 4. Calibration

For calibrating the arm's position:

1. Disable torque using the "Torque OFF" command
2. Manually move the arm to a known reference position
3. Record the position as a calibration point
4. Re-enable torque using the "Torque ON" command

## Integration with Other Control Features

The Torque Lock Control system integrates with other RoArm-M3 Pro control features:

### 1. AngleCtrl (Servo Angle Control)

- When a new AngleCtrl command is received, torque is automatically re-enabled
- This ensures that the arm can execute the commanded movement
- After movement completion, torque remains enabled to maintain position

### 2. DEFA (Dynamic External Force Adaptive Control)

- DEFA requires torque to be enabled to function
- When torque is disabled, DEFA is automatically disabled
- When DEFA is enabled, torque is automatically enabled

### 3. COORDCTRL (End Point Coordinate Control)

- Similar to AngleCtrl, COORDCTRL commands automatically re-enable torque
- This ensures that the arm can execute the commanded movement
- After movement completion, torque remains enabled to maintain position

## Performance Considerations

### 1. Power Consumption

- When torque is enabled, power consumption is higher due to active holding
- For battery-powered applications, consider disabling torque when not needed
- Power consumption varies based on the load and position of each joint

### 2. Heat Generation

- Active torque generates heat in the servo motors
- Extended periods of high torque may lead to increased temperatures
- Consider monitoring servo temperatures during prolonged use

### 3. Mechanical Wear

- Frequent toggling of torque may increase mechanical wear
- For applications requiring frequent torque changes, consider using lower speeds
- Metal shell servos (ST3235) are more durable for frequent torque cycling

## Troubleshooting

### Common Issues

1. **Joints Move Unexpectedly After Torque Off**
   - **Symptom**: Joints drift or move after disabling torque
   - **Possible Causes**: Gravity, unbalanced load, mechanical issues
   - **Solutions**: Ensure balanced load distribution, check for mechanical binding

2. **Torque Doesn't Disable**
   - **Symptom**: Joints still resist movement after "Torque OFF" command
   - **Possible Causes**: Communication issues, command not received
   - **Solutions**: Verify command format, check connections, restart system

3. **Torque Re-enables Unexpectedly**
   - **Symptom**: Torque turns back on without explicit command
   - **Possible Causes**: Other control commands sent, automatic re-engagement
   - **Solutions**: Check for conflicting commands, be aware of automatic re-engagement behavior

4. **Excessive Vibration When Torque Enabled**
   - **Symptom**: Arm vibrates or oscillates when torque is enabled
   - **Possible Causes**: PID tuning issues, mechanical resonance
   - **Solutions**: Reduce load, check for mechanical issues, update firmware

## Safety Considerations

1. **Gravity Compensation**
   - When torque is disabled, the arm will move due to gravity
   - Be prepared to support the arm when disabling torque
   - Consider the arm's configuration before disabling torque

2. **Pinch Points**
   - Be aware of potential pinch points when manually positioning the arm
   - Keep hands clear of joints when re-enabling torque
   - Ensure adequate clearance around the arm

3. **Unexpected Re-engagement**
   - Be aware that torque will automatically re-enable when new movement commands are received
   - This can cause unexpected movement if the arm is being manually positioned
   - Always maintain control of the arm when torque is disabled

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

The Torque Lock Control feature is a versatile and powerful capability of the RoArm-M3 Pro robotic arm. By enabling users to toggle between active position maintenance and free manual positioning, it opens up a wide range of applications from teaching by demonstration to emergency intervention. Understanding how to effectively use this feature and its integration with other control systems is essential for maximizing the potential of the RoArm-M3 Pro in various robotic applications.
