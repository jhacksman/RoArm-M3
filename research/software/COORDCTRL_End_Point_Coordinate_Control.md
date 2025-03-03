# COORDCTRL: End Point Coordinate Control for RoArm-M3 Pro

## Overview

COORDCTRL (End Point Coordinate Control) is an advanced control feature of the RoArm-M3 Pro robotic arm that enables users to control the position of the end-effector (gripper) in Cartesian space (X, Y, Z coordinates) rather than controlling individual joint angles. This feature simplifies complex movements by allowing users to specify the desired position of the end-effector in a more intuitive coordinate system, while the arm's internal inverse kinematics algorithms automatically calculate the required joint angles to achieve that position. This capability is essential for applications requiring precise positioning in three-dimensional space, such as pick-and-place operations, drawing, and object manipulation.

![COORDCTRL Web Interface](https://www.waveshare.com/w/upload/9/95/500px-RAM3-web1%282%29.png)

## Key Features

- **Cartesian Coordinate Control**: Control the end-effector position using X, Y, Z coordinates
- **Inverse Kinematics**: Automatically calculates joint angles required to reach specified coordinates
- **Intuitive Positioning**: Simplifies complex movements by abstracting joint-level control
- **Web Interface Control**: Interactive coordinate control through the web-based interface
- **JSON Command Control**: Programmable through standardized JSON commands
- **Speed Control**: Adjustable movement speed for coordinate transitions
- **Workspace Awareness**: Respects the physical limitations of the arm's workspace
- **Multiple Control Methods**: Accessible via web interface, JSON commands, and programmatic APIs

## Technical Implementation

### Coordinate System

The COORDCTRL system uses a Cartesian coordinate system with the following conventions:

1. **Origin**: Located at the center of the base of the robotic arm
2. **X-Axis**: Forward/backward direction relative to the front of the arm
3. **Y-Axis**: Left/right direction relative to the front of the arm
4. **Z-Axis**: Up/down direction perpendicular to the base
5. **Units**: Millimeters (mm) for all coordinate values

### Inverse Kinematics

The core of the COORDCTRL system is the inverse kinematics algorithm that translates end-effector coordinates into joint angles:

1. **Mathematical Model**: Uses a mathematical model of the arm's geometry
2. **Joint Constraints**: Accounts for the physical limitations of each joint
3. **Singularity Handling**: Manages singularities (configurations where multiple joint solutions exist)
4. **Optimization**: Selects the most efficient joint configuration when multiple solutions exist
5. **Error Handling**: Detects when requested coordinates are outside the reachable workspace

### Control Architecture

The COORDCTRL system operates through a hierarchical control architecture:

1. **User Interface Layer**: Web interface, Python API, or custom applications
2. **Command Processing Layer**: ESP32 microcontroller processes coordinate commands
3. **Inverse Kinematics Layer**: Calculates joint angles from coordinates
4. **Joint Control Layer**: Translates joint angles to servo commands
5. **Servo Control Layer**: Manages the physical movement of each servo

## Usage Methods

### 1. Web Interface Control

The web interface provides intuitive control of the end-effector position:

1. Connect to the RoArm-M3 Pro's WiFi (SSID: RoArm-M3, Password: 12345678)
2. Navigate to 192.168.4.1 in a web browser
3. Use the COORDCTRL section:
   - Enter the desired X, Y, Z coordinates
   - Set the movement speed
   - Click "Send" to move the arm to the specified position

### 2. JSON Command Control

Direct control via JSON commands allows for programmatic coordinate control:

```json
{
  "type": "CoordCtrl",
  "x": 150,
  "y": 0,
  "z": 100,
  "speed": 50
}
```

Parameters:
- `type`: Command type (CoordCtrl)
- `x`: X-coordinate in millimeters
- `y`: Y-coordinate in millimeters
- `z`: Z-coordinate in millimeters
- `speed`: Movement speed (0-100, where 0 is maximum speed)

### 3. Python API Control

Control through Python scripts using either HTTP or Serial communication:

#### HTTP Example:
```python
import requests
import json

def coord_control(ip_address, x, y, z, speed):
    cmd = {
        "type": "CoordCtrl",
        "x": x,
        "y": y,
        "z": z,
        "speed": speed
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    return response.text

# Move end-effector to position (150, 0, 100) at 50% speed
coord_control("192.168.4.1", 150, 0, 100, 50)
```

#### Serial Example:
```python
import serial
import json

def coord_control(port, x, y, z, speed):
    cmd = {
        "type": "CoordCtrl",
        "x": x,
        "y": y,
        "z": z,
        "speed": speed
    }
    with serial.Serial(port, baudrate=115200) as ser:
        ser.write(json.dumps(cmd).encode() + b'\n')

# Move end-effector to position (150, 0, 100) at 50% speed
coord_control("/dev/ttyUSB0", 150, 0, 100, 50)
```

## Workspace Limitations

The RoArm-M3 Pro has a physical workspace with the following approximate dimensions:

1. **X-Axis Range**: Approximately -200mm to 200mm
2. **Y-Axis Range**: Approximately -200mm to 200mm
3. **Z-Axis Range**: Approximately 0mm to 300mm
4. **Effective Radius**: Approximately 320mm from the base
5. **Workspace Volume**: Approximately 0.027 cubic meters

Note that these values are approximate and the actual reachable workspace is not a perfect cube or sphere due to the arm's geometry and joint limitations.

## Common Use Cases

### 1. Pick and Place Operations

For applications requiring object manipulation:

1. Move to approach position above the object
2. Lower to the object position
3. Close gripper
4. Move to approach position above the destination
5. Lower to the destination position
6. Open gripper
7. Return to home position

Example sequence:
```python
# Approach object
coord_control("192.168.4.1", 150, 0, 150, 50)
# Lower to object
coord_control("192.168.4.1", 150, 0, 50, 30)
# Close gripper (using AngleCtrl)
angle_control("192.168.4.1", 6, 3.14, 50)
# Lift object
coord_control("192.168.4.1", 150, 0, 150, 50)
# Move to destination approach
coord_control("192.168.4.1", 0, 150, 150, 50)
# Lower to destination
coord_control("192.168.4.1", 0, 150, 50, 30)
# Open gripper (using AngleCtrl)
angle_control("192.168.4.1", 6, 1.57, 50)
# Return to home
coord_control("192.168.4.1", 150, 0, 150, 50)
```

### 2. Drawing and Tracing

For applications requiring the end-effector to follow a specific path:

1. Move to the starting position
2. Lower to the drawing surface
3. Follow a series of coordinates to create the desired pattern
4. Lift from the drawing surface
5. Return to home position

Example sequence for drawing a square:
```python
# Move to starting position
coord_control("192.168.4.1", 100, 100, 50, 50)
# Draw first line
coord_control("192.168.4.1", 100, -100, 50, 30)
# Draw second line
coord_control("192.168.4.1", -100, -100, 50, 30)
# Draw third line
coord_control("192.168.4.1", -100, 100, 50, 30)
# Draw fourth line (complete square)
coord_control("192.168.4.1", 100, 100, 50, 30)
# Lift from drawing surface
coord_control("192.168.4.1", 100, 100, 150, 50)
```

### 3. Workspace Mapping

For applications requiring knowledge of the arm's capabilities:

1. Systematically move the end-effector through a grid of coordinates
2. Record successful positions to map the reachable workspace
3. Use this data to plan future movements within the confirmed workspace

Example grid mapping approach:
```python
def map_workspace(ip_address, grid_size=50, speed=30):
    workspace_map = []
    for x in range(-200, 201, grid_size):
        for y in range(-200, 201, grid_size):
            for z in range(0, 301, grid_size):
                try:
                    response = coord_control(ip_address, x, y, z, speed)
                    # Check if movement was successful
                    if "success" in response:
                        workspace_map.append((x, y, z))
                except Exception as e:
                    # Movement failed or timed out
                    pass
    return workspace_map
```

## Integration with Other Control Features

The COORDCTRL system integrates with other RoArm-M3 Pro control features:

### 1. AngleCtrl (Servo Angle Control)

- COORDCTRL and AngleCtrl can be used interchangeably
- Switching between the two control modes is seamless
- The last command (whether COORDCTRL or AngleCtrl) takes precedence

### 2. Torque Lock Control

- COORDCTRL requires torque to be enabled to function
- When torque is disabled, COORDCTRL commands will automatically re-enable torque
- After COORDCTRL movement completion, torque remains enabled to maintain position

### 3. DEFA (Dynamic External Force Adaptive Control)

- When DEFA is enabled, the arm will return to the last COORDCTRL position after being manually moved
- COORDCTRL commands update the position memory used by DEFA
- This allows for maintaining end-effector positions in Cartesian space despite disturbances

## Performance Considerations

### 1. Movement Speed

- Higher speed values result in slower, more controlled movements
- Speed value of 0 uses maximum speed
- Consider using moderate speeds (40-60) for precise positioning
- Very slow speeds (80-100) are useful for delicate operations

### 2. Movement Accuracy

- Accuracy decreases at the extremes of the workspace
- Best accuracy is achieved in the central region of the workspace
- Consider the arm's configuration when planning precise movements
- Multiple approaches to the same position may yield slightly different results

### 3. Singularities

- Certain arm configurations have multiple possible joint solutions
- The arm may take unexpected paths when moving near singularities
- To avoid unpredictable movements, plan paths that avoid singularities
- Break complex movements into smaller steps when necessary

## Troubleshooting

### Common Issues

1. **Unreachable Coordinates**
   - **Symptom**: Arm doesn't move or moves to an incorrect position
   - **Possible Causes**: Coordinates outside workspace, singularity, mechanical limitations
   - **Solutions**: Verify coordinates are within workspace, try different approach paths

2. **Jerky Movement**
   - **Symptom**: Arm moves in an uneven or jerky manner
   - **Possible Causes**: Speed set too low, passing through singularities
   - **Solutions**: Increase speed value, plan paths that avoid singularities

3. **Position Drift**
   - **Symptom**: End-effector doesn't maintain its position under load
   - **Possible Causes**: Torque limit reached, servo overheating
   - **Solutions**: Reduce load, check power supply, enable DEFA for position maintenance

4. **Unexpected Paths**
   - **Symptom**: Arm takes unexpected routes between coordinates
   - **Possible Causes**: Inverse kinematics selecting different solutions, singularities
   - **Solutions**: Break movements into smaller steps, specify intermediate coordinates

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

## Advanced Usage

### 1. Path Planning

For complex movements, consider implementing path planning algorithms:

```python
def linear_path(start, end, steps=10):
    """Generate a linear path between two points with a specified number of steps."""
    path = []
    for i in range(steps + 1):
        t = i / steps
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        z = start[2] + t * (end[2] - start[2])
        path.append((x, y, z))
    return path

def follow_path(ip_address, path, speed):
    """Move the arm along a specified path."""
    for point in path:
        x, y, z = point
        coord_control(ip_address, x, y, z, speed)
        time.sleep(0.5)  # Wait for movement to complete

# Example usage
start_point = (150, 0, 150)
end_point = (0, 150, 50)
path = linear_path(start_point, end_point, steps=20)
follow_path("192.168.4.1", path, 70)
```

### 2. Workspace Visualization

For better understanding of the arm's capabilities:

```python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_workspace(workspace_map):
    """Visualize the mapped workspace in 3D."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x = [point[0] for point in workspace_map]
    y = [point[1] for point in workspace_map]
    z = [point[2] for point in workspace_map]
    
    ax.scatter(x, y, z, c='b', marker='o')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('RoArm-M3 Pro Workspace')
    
    plt.show()

# Example usage
workspace = map_workspace("192.168.4.1")
visualize_workspace(workspace)
```

## Conclusion

The COORDCTRL (End Point Coordinate Control) feature is a powerful capability of the RoArm-M3 Pro robotic arm that simplifies complex movements by allowing users to control the end-effector position in an intuitive Cartesian coordinate system. By abstracting the complexity of joint-level control and automatically handling inverse kinematics, this feature enables a wide range of applications from simple pick-and-place operations to complex path following. Understanding the workspace limitations, integration with other control features, and performance considerations is essential for effectively utilizing this capability in various robotic applications.
