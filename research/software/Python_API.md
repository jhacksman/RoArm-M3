# Python API

## Overview

The Python API for the RoArm-M3 Pro robotic arm provides a comprehensive set of libraries and functions that enable programmatic control of the arm using the Python programming language. This API serves as a bridge between custom Python applications and the robotic arm's control system, allowing for sophisticated automation, integration with other software systems, and development of complex robotic applications without requiring low-level programming knowledge.

![Python API](https://www.waveshare.com/w/upload/thumb/a/a3/RoArm-M3_details_04.jpg/400px-RoArm-M3_details_04.jpg)

## Key Features

- **High-Level Control**: Abstract, intuitive functions for controlling the arm
- **Multiple Communication Methods**: Support for both HTTP and Serial communication
- **JSON Command Integration**: Seamless translation of Python functions to JSON commands
- **Comprehensive Motion Control**: Functions for joint, coordinate, and sequence control
- **Error Handling**: Robust exception handling and error reporting
- **Cross-Platform Compatibility**: Works on Windows, macOS, and Linux
- **Minimal Dependencies**: Requires only standard Python libraries
- **Extensible Design**: Easily customizable for specific applications
- **Documentation**: Comprehensive docstrings and examples
- **Integration Support**: Designed to work with other Python frameworks and libraries

## Installation

The Python API can be installed using pip:

```bash
pip install roarm-m3
```

Alternatively, you can install from source:

```bash
git clone https://github.com/waveshare/RoArm-M3-Python.git
cd RoArm-M3-Python
pip install -e .
```

### Dependencies

The API has minimal dependencies:

- Python 3.6+
- requests (for HTTP communication)
- pyserial (for Serial communication)
- json (standard library)

## Connection Setup

The Python API supports two primary methods of communication with the RoArm-M3 Pro:

### HTTP Connection

```python
from roarm_m3 import RoArmHTTP

# Connect to the arm via WiFi
arm = RoArmHTTP(ip="192.168.4.1")

# Test connection
if arm.is_connected():
    print("Successfully connected to RoArm-M3 Pro")
else:
    print("Connection failed")
```

### Serial Connection

```python
from roarm_m3 import RoArmSerial

# Connect to the arm via USB
arm = RoArmSerial(port="/dev/ttyUSB0", baudrate=115200)

# Test connection
if arm.is_connected():
    print("Successfully connected to RoArm-M3 Pro")
else:
    print("Connection failed")
```

## Basic Usage

### Joint Control

Control individual servo joints:

```python
from roarm_m3 import RoArmHTTP
import time

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Move joint 1 to 90 degrees at 50% speed
arm.move_joint(joint_id=1, angle=90, speed=50)

# Move multiple joints simultaneously
arm.move_joints({
    1: 90,  # Joint 1 to 90 degrees
    2: 45,  # Joint 2 to 45 degrees
    3: 120  # Joint 3 to 120 degrees
}, speed=50)

# Wait for movements to complete
time.sleep(2)

# Get current joint positions
positions = arm.get_joint_positions()
print(f"Current positions: {positions}")
```

### Coordinate Control

Position the end-effector using Cartesian coordinates:

```python
from roarm_m3 import RoArmHTTP

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Move to a specific position in 3D space
arm.move_to_position(x=150, y=0, z=100, speed=50)

# Move with orientation control
arm.move_to_position(
    x=150, y=0, z=100,
    roll=0, pitch=90, yaw=0,
    speed=50
)

# Get current end-effector position
position = arm.get_position()
print(f"Current position: {position}")
```

### Gripper Control

Control the gripper (servo 6):

```python
from roarm_m3 import RoArmHTTP

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Open gripper (0 degrees = fully open)
arm.set_gripper(angle=0, speed=50)

# Close gripper (180 degrees = fully closed)
arm.set_gripper(angle=180, speed=50)

# Set gripper to half-open position
arm.set_gripper(angle=90, speed=50)
```

### Sequence Control

Create and run motion sequences:

```python
from roarm_m3 import RoArmHTTP
import time

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Create a new sequence
sequence = arm.create_sequence("pick_and_place")

# Add waypoints to the sequence
arm.move_joint(1, 90)
arm.move_joint(2, 45)
time.sleep(1)
sequence.add_current_position("position1")

arm.move_joint(1, 45)
arm.move_joint(2, 90)
time.sleep(1)
sequence.add_current_position("position2")

# Save the sequence
sequence.save()

# Run the sequence
arm.run_sequence("pick_and_place", loop=False, speed=70)

# Wait for sequence to complete
time.sleep(5)

# Stop a running sequence
arm.stop_sequence()
```

## Advanced Features

### Custom JSON Commands

Send custom JSON commands directly:

```python
from roarm_m3 import RoArmHTTP

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Send a custom JSON command
custom_command = {
    "type": "MultiCmd",
    "cmds": [
        {"type": "AngleCtrl", "id": 1, "angle": 90, "speed": 50},
        {"type": "Delay", "time": 1000},
        {"type": "AngleCtrl", "id": 2, "angle": 45, "speed": 50}
    ]
}

response = arm.send_command(custom_command)
print(f"Command response: {response}")
```

### Leader-Follower Mode

Configure leader-follower mode:

```python
from roarm_m3 import RoArmHTTP

# Connect to the leader arm
leader_arm = RoArmHTTP(ip="192.168.4.1")

# Set as leader
leader_arm.set_mode("leader")

# Connect to the follower arm
follower_arm = RoArmHTTP(ip="192.168.4.2")

# Set as follower, specifying the leader's MAC address
follower_arm.set_mode("follower", leader_mac="AA:BB:CC:DD:EE:FF")

# Now any movement of the leader arm will be mirrored by the follower
```

### Event Callbacks

Register callbacks for arm events:

```python
from roarm_m3 import RoArmHTTP

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Define callback functions
def on_position_changed(position):
    print(f"Position changed: {position}")

def on_error(error):
    print(f"Error occurred: {error}")

# Register callbacks
arm.on_position_changed(on_position_changed)
arm.on_error(on_error)

# Start event monitoring
arm.start_monitoring()

# Perform some movements
arm.move_joint(1, 90)

# Stop event monitoring when done
arm.stop_monitoring()
```

### Path Planning

Create complex motion paths:

```python
from roarm_m3 import RoArmHTTP
import numpy as np

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Define a circular path
radius = 50
center_x, center_y = 150, 0
center_z = 100
points = []

for angle in np.linspace(0, 2*np.pi, 20):
    x = center_x + radius * np.cos(angle)
    y = center_y + radius * np.sin(angle)
    z = center_z
    points.append((x, y, z))

# Execute the path
for x, y, z in points:
    arm.move_to_position(x, y, z, speed=30)
    # Small delay between points
    arm.wait(0.1)
```

## API Reference

### Core Classes

#### `RoArmBase`

Base class for RoArm communication.

```python
class RoArmBase:
    def is_connected(self):
        """Check if connected to the arm."""
        
    def send_command(self, command):
        """Send a JSON command to the arm."""
        
    def get_status(self):
        """Get the current status of the arm."""
```

#### `RoArmHTTP`

HTTP communication interface.

```python
class RoArmHTTP(RoArmBase):
    def __init__(self, ip="192.168.4.1", port=80):
        """Initialize HTTP connection."""
```

#### `RoArmSerial`

Serial communication interface.

```python
class RoArmSerial(RoArmBase):
    def __init__(self, port, baudrate=115200, timeout=1):
        """Initialize serial connection."""
```

#### `Sequence`

Sequence management class.

```python
class Sequence:
    def add_position(self, name, position):
        """Add a position to the sequence."""
        
    def add_current_position(self, name):
        """Add the current arm position to the sequence."""
        
    def save(self):
        """Save the sequence to the arm."""
```

### Movement Functions

```python
def move_joint(self, joint_id, angle, speed=50):
    """Move a single joint to the specified angle."""
    
def move_joints(self, joint_angles, speed=50):
    """Move multiple joints simultaneously."""
    
def move_to_position(self, x, y, z, roll=0, pitch=0, yaw=0, speed=50):
    """Move the end-effector to a position in 3D space."""
    
def move_linear(self, x, y, z, speed=50, steps=10):
    """Move in a straight line to the target position."""
    
def set_gripper(self, angle, speed=50):
    """Set the gripper position."""
```

### Sequence Functions

```python
def create_sequence(self, name):
    """Create a new sequence."""
    
def run_sequence(self, name, loop=False, speed=50):
    """Run a saved sequence."""
    
def stop_sequence(self):
    """Stop the currently running sequence."""
    
def get_sequences(self):
    """Get a list of saved sequences."""
```

### System Functions

```python
def reset(self):
    """Reset the arm to its default position."""
    
def set_mode(self, mode, **kwargs):
    """Set the operation mode of the arm."""
    
def get_joint_positions(self):
    """Get the current joint positions."""
    
def get_position(self):
    """Get the current end-effector position."""
    
def wait(self, seconds):
    """Wait for the specified number of seconds."""
```

## Integration Examples

### Integration with OpenCV

```python
import cv2
import numpy as np
from roarm_m3 import RoArmHTTP

# Connect to the arm
arm = RoArmHTTP(ip="192.168.4.1")

# Initialize camera
cap = cv2.VideoCapture(0)

# Define color range for object detection (e.g., red object)
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

while True:
    # Capture frame
    ret, frame = cap.read()
    if not ret:
        break
        
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create mask for red color
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get centroid
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Draw centroid
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            
            # Map camera coordinates to arm coordinates
            # This is a simplified example - actual mapping would require calibration
            arm_x = 150 + (cx - 320) * 0.5
            arm_y = (cy - 240) * 0.5
            arm_z = 100
            
            # Move arm to position
            arm.move_to_position(arm_x, arm_y, arm_z, speed=30)
    
    # Display frame
    cv2.imshow('Frame', frame)
    
    # Exit on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
arm.reset()
```

### Integration with ROS2

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import json
from roarm_m3 import RoArmHTTP

class RoArmNode(Node):
    def __init__(self):
        super().__init__('roarm_node')
        
        # Connect to the arm
        self.arm = RoArmHTTP(ip="192.168.4.1")
        
        # Create subscribers
        self.pose_subscription = self.create_subscription(
            Pose,
            'roarm/target_pose',
            self.pose_callback,
            10)
            
        self.command_subscription = self.create_subscription(
            String,
            'roarm/command',
            self.command_callback,
            10)
            
        # Create publishers
        self.status_publisher = self.create_publisher(
            String,
            'roarm/status',
            10)
            
        # Create timer for status updates
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('RoArm ROS2 node initialized')
        
    def pose_callback(self, msg):
        # Extract position from pose message
        x = msg.position.x * 1000  # Convert to mm
        y = msg.position.y * 1000
        z = msg.position.z * 1000
        
        # Extract orientation (simplified)
        # In a real application, convert quaternion to Euler angles
        roll = 0
        pitch = 90
        yaw = 0
        
        # Move arm to position
        self.arm.move_to_position(x, y, z, roll, pitch, yaw)
        self.get_logger().info(f'Moving to position: ({x}, {y}, {z})')
        
    def command_callback(self, msg):
        # Parse command from JSON string
        try:
            command = json.loads(msg.data)
            response = self.arm.send_command(command)
            self.get_logger().info(f'Command executed: {response}')
        except Exception as e:
            self.get_logger().error(f'Command execution failed: {str(e)}')
            
    def publish_status(self):
        # Get arm status
        status = self.arm.get_status()
        
        # Publish status as JSON string
        msg = String()
        msg.data = json.dumps(status)
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Available Resources

### Documentation
- [RoArm-M3 User Manual](https://www.waveshare.com/wiki/RoArm-M3)
- [Python API Documentation](https://www.waveshare.com/wiki/RoArm-M3)
- [JSON Command Reference](https://www.waveshare.com/wiki/RoArm-M3)

### Software
- [Python SDK GitHub Repository](https://github.com/waveshare/RoArm-M3-Python)
- [Example Scripts](https://www.waveshare.com/wiki/RoArm-M3)
- [Demo Applications](https://www.waveshare.com/wiki/RoArm-M3)

### Related Components
- [JSON Command System](https://www.waveshare.com/wiki/RoArm-M3)
- [Web Interface](https://www.waveshare.com/wiki/RoArm-M3)
- [ROS2 Integration](https://www.waveshare.com/wiki/RoArm-M3)

## Troubleshooting

### Common Issues

1. **Connection Failures**
   - **Symptom**: Unable to connect to the arm
   - **Possible Causes**:
     - Incorrect IP address or serial port
     - Arm not powered on
     - WiFi connection issues
     - USB driver problems
   - **Solutions**:
     - Verify the arm is powered on
     - Check WiFi connection to the arm's network
     - Confirm IP address (default: 192.168.4.1)
     - For serial connection, check port name and permissions
     - Install appropriate USB drivers if needed

2. **Command Execution Errors**
   - **Symptom**: Commands fail to execute or return errors
   - **Possible Causes**:
     - Invalid command parameters
     - Arm in error state
     - Position unreachable
     - Communication timeout
   - **Solutions**:
     - Check parameter values against allowed ranges
     - Reset the arm to clear error states
     - Verify target positions are within the arm's workspace
     - Implement timeout handling and retry logic

3. **Performance Issues**
   - **Symptom**: Slow response or jerky movements
   - **Possible Causes**:
     - Network latency
     - Too many commands sent too quickly
     - Resource constraints on the controlling device
   - **Solutions**:
     - Reduce command frequency
     - Use batch commands where possible
     - Optimize code for performance
     - Consider using serial connection for time-critical applications

4. **Integration Problems**
   - **Symptom**: Difficulties integrating with other systems
   - **Possible Causes**:
     - Version incompatibilities
     - Missing dependencies
     - Threading issues
   - **Solutions**:
     - Check API version compatibility
     - Install all required dependencies
     - Use proper thread synchronization
     - Consider using the event callback system

## Best Practices

1. **Connection Management**
   - Initialize connection at startup
   - Implement reconnection logic
   - Close connection properly when done
   - Handle connection errors gracefully

2. **Error Handling**
   - Always check command responses
   - Implement try-except blocks around API calls
   - Log errors for troubleshooting
   - Provide meaningful error messages to users

3. **Performance Optimization**
   - Batch related commands when possible
   - Use appropriate speeds for different operations
   - Implement rate limiting for command sequences
   - Consider arm dynamics when planning movements

4. **Resource Management**
   - Release resources when not in use
   - Close connections properly
   - Implement proper threading if using callbacks
   - Monitor memory usage for long-running applications

5. **Safety Considerations**
   - Implement emergency stop functionality
   - Validate movement commands before sending
   - Consider workspace limitations
   - Test movements at low speeds first

## Conclusion

The Python API for the RoArm-M3 Pro robotic arm provides a powerful and flexible interface for controlling the arm programmatically. With support for both HTTP and Serial communication, comprehensive motion control functions, and easy integration with other Python libraries and frameworks, it enables the development of sophisticated robotic applications ranging from simple automation tasks to complex computer vision-based systems. The API's intuitive design, extensive documentation, and robust error handling make it accessible to both beginners and experienced developers, while its extensible architecture allows for customization to meet specific application requirements.
