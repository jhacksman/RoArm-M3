# RoArm-M3 Pro and LeRobot Interface

This documentation explains how to interface the RoArm-M3 Pro robotic arm with the LeRobot AI framework. It covers the necessary setup, configuration, and communication methods to enable AI-driven control of the arm.

## Interface Overview

The RoArm-M3 Pro can interface with LeRobot through several methods:

1. **HTTP Communication**: Using the arm's web server for command transmission
2. **Serial Communication**: Direct connection via USB for lower latency control
3. **ROS2 Bridge**: Integration through ROS2 for advanced robotics applications
4. **Python API**: High-level Python interface for simplified control

Each method has its advantages and is suitable for different use cases.

## System Architecture

The interface between RoArm-M3 Pro and LeRobot follows this general architecture:

```
┌─────────────┐     ┌───────────────┐     ┌────────────────┐
│  LeRobot    │     │ Communication │     │  RoArm-M3 Pro  │
│  Framework  ├────►│    Layer      ├────►│  Robotic Arm   │
└─────────────┘     └───────────────┘     └────────────────┘
       ▲                                          │
       │                                          │
       └──────────────────────────────────────────┘
                    Feedback Loop
```

- **LeRobot Framework**: Provides AI models and control algorithms
- **Communication Layer**: Handles data transmission between LeRobot and the arm
- **RoArm-M3 Pro**: Executes commands and provides feedback
- **Feedback Loop**: Enables closed-loop control and learning

## Installation

### 1. Install LeRobot Framework

```bash
# Create a virtual environment
python -m venv lerobot_env
source lerobot_env/bin/activate  # On Windows: lerobot_env\Scripts\activate

# Install LeRobot
pip install lerobot

# Install additional dependencies
pip install numpy torch pyserial requests
```

### 2. Configure RoArm-M3 Pro

Ensure your RoArm-M3 Pro is set up and operational:

1. Power on the arm
2. Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678)
3. Access the web interface at 192.168.4.1
4. Verify that the arm is functioning correctly

### 3. Test Communication

Test the communication between LeRobot and the RoArm-M3 Pro:

```python
import lerobot
import requests
import json

# Define the arm's IP address
arm_ip = "192.168.4.1"

# Test HTTP communication
def test_connection():
    cmd = {
        "type": "GetStatus"
    }
    url = f"http://{arm_ip}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Connection test response: {response.text}")

test_connection()
```

## Communication Methods

### HTTP Communication

HTTP communication is the simplest method for interfacing with the RoArm-M3 Pro:

```python
import requests
import json

def send_command(ip_address, command):
    url = f"http://{ip_address}/js?json={json.dumps(command)}"
    response = requests.get(url)
    return response.text

# Example: Move a joint
command = {
    "type": "AngleCtrl",
    "id": 1,
    "angle": 1.57,  # 90 degrees in radians
    "speed": 50
}

result = send_command("192.168.4.1", command)
print(result)
```

### Serial Communication

Serial communication provides lower latency for real-time control:

```python
import serial
import json
import time

def send_serial_command(port, command):
    with serial.Serial(port, baudrate=115200) as ser:
        cmd_str = json.dumps(command) + '\n'
        ser.write(cmd_str.encode())
        time.sleep(0.1)  # Give time for response
        response = ser.readline().decode().strip()
        return response

# Example: Move a joint
command = {
    "type": "AngleCtrl",
    "id": 1,
    "angle": 1.57,  # 90 degrees in radians
    "speed": 50
}

result = send_serial_command("/dev/ttyUSB0", command)  # Adjust port as needed
print(result)
```

### ROS2 Bridge

For advanced applications, you can use ROS2 as a bridge between LeRobot and the RoArm-M3 Pro:

1. Install ROS2 on your system
2. Set up the RoArm-M3 ROS2 package (see [ROS2 Integration](../../research/software/ROS2_Integration.md))
3. Use LeRobot's ROS2 interface to communicate with the arm

### Python API

The RoArm-M3 Pro Python API provides a high-level interface:

```python
from roarm_m3 import RoArmM3

# Initialize the arm
arm = RoArmM3(connection_type="http", address="192.168.4.1")

# Connect to the arm
arm.connect()

# Move a joint
arm.move_joint(joint_id=1, angle=90, speed=50)

# Disconnect
arm.disconnect()
```

## LeRobot Integration

### Recording Datasets

To record datasets for training:

```python
import lerobot
from lerobot.data import DataRecorder
from roarm_m3 import RoArmM3

# Initialize the arm
arm = RoArmM3(connection_type="http", address="192.168.4.1")
arm.connect()

# Initialize the data recorder
recorder = DataRecorder(output_dir="./dataset")

# Start recording
recorder.start()

# Control the arm manually or through a predefined sequence
# The recorder will capture joint positions, sensor data, etc.

# Stop recording
recorder.stop()

# Disconnect
arm.disconnect()
```

### Training Models

Train a model on the recorded dataset:

```python
import lerobot
from lerobot.models import ImitationLearningModel
from lerobot.data import Dataset

# Load the dataset
dataset = Dataset.from_directory("./dataset")

# Initialize the model
model = ImitationLearningModel()

# Train the model
model.train(dataset, epochs=100)

# Save the model
model.save("./trained_model")
```

### Deploying Models

Deploy the trained model to control the arm:

```python
import lerobot
from lerobot.models import ImitationLearningModel
from roarm_m3 import RoArmM3

# Load the trained model
model = ImitationLearningModel.load("./trained_model")

# Initialize the arm
arm = RoArmM3(connection_type="http", address="192.168.4.1")
arm.connect()

# Deploy the model
while True:
    # Get the current state
    state = arm.get_state()
    
    # Predict the next action
    action = model.predict(state)
    
    # Execute the action
    arm.execute_action(action)
    
    # Optional: Break condition
    if done_condition:
        break

# Disconnect
arm.disconnect()
```

## Troubleshooting

### Common Issues

1. **Connection Failures**
   - Verify the arm is powered on
   - Check WiFi connection
   - Ensure correct IP address
   - Try different communication methods

2. **Command Execution Errors**
   - Verify command format
   - Check for joint limits
   - Ensure the arm is not in an error state

3. **Performance Issues**
   - Use serial communication for lower latency
   - Optimize model inference
   - Reduce control loop frequency if necessary

### Debugging Tools

1. **Status Queries**
   ```python
   status_cmd = {"type": "GetStatus"}
   status = send_command("192.168.4.1", status_cmd)
   print(status)
   ```

2. **Error Logging**
   ```python
   import logging
   logging.basicConfig(level=logging.DEBUG)
   ```

3. **Communication Testing**
   ```python
   # Test different communication methods
   test_http()
   test_serial()
   ```

## Advanced Configuration

For advanced configuration options, refer to:
- [LeRobot Configuration](./configuration.md)
- [RoArm-M3 Pro Advanced Settings](../../research/software/Web_Interface.md)

## Next Steps

After setting up the interface:
1. Explore the [example projects](../examples/)
2. Develop your own applications
3. Contribute to the LeRobot community
