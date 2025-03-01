# JSON Command System

## Overview

The JSON Command System is a comprehensive protocol for controlling the RoArm-M3 Pro robotic arm through structured JSON (JavaScript Object Notation) commands. This system provides a standardized, text-based interface for programmatic control of all arm functions, enabling integration with various programming languages and control systems. The JSON Command System serves as the foundation for both the web interface and API interactions, offering a flexible and powerful method for precise control of the robotic arm.

![JSON Command System](https://www.waveshare.com/w/upload/thumb/a/a3/RoArm-M3_details_04.jpg/400px-RoArm-M3_details_04.jpg)

## Key Features

- **Standardized Format**: Consistent JSON structure for all commands
- **Multiple Transmission Methods**: Send via web interface, HTTP requests, or serial communication
- **Comprehensive Control**: Access to all arm functions and features
- **Real-Time Execution**: Immediate command processing and execution
- **Error Handling**: Structured error responses and validation
- **Command Chaining**: Execute multiple commands in sequence
- **Parameter Flexibility**: Support for various data types and control parameters
- **Cross-Platform Compatibility**: Works with any system capable of generating JSON
- **Extensible Design**: Supports addition of new command types
- **Feedback Mechanism**: Returns status and position information

## Command Structure

All commands in the JSON Command System follow a consistent structure:

```json
{
  "type": "CommandType",
  "param1": value1,
  "param2": value2,
  ...
}
```

### Basic Structure Elements

1. **Command Type**:
   - The `"type"` field specifies the command category and action
   - Required in all commands
   - Case-sensitive string value

2. **Parameters**:
   - Command-specific parameters with appropriate data types
   - Optional or required depending on the command type
   - May include nested objects or arrays

3. **Response Format**:
   - Commands typically return a response in JSON format
   - Includes success/failure status and relevant data
   - May contain error messages and codes

### Example Basic Command

```json
{
  "type": "AngleCtrl",
  "id": 1,
  "angle": 90,
  "speed": 50
}
```

This command moves servo ID 1 to a 90-degree position at 50% speed.

### Example Response

```json
{
  "status": "success",
  "data": {
    "id": 1,
    "angle": 90,
    "position": 2048
  }
}
```

## Command Categories

The JSON Command System includes several categories of commands for different aspects of arm control:

### 1. Servo Control Commands

Commands for controlling individual servos:

#### AngleCtrl - Control Servo Angle

```json
{
  "type": "AngleCtrl",
  "id": 1,
  "angle": 90,
  "speed": 50
}
```

Parameters:
- `id`: Servo ID (1-6)
- `angle`: Target angle in degrees (0-180)
- `speed`: Movement speed (1-100)

#### TorqueCtrl - Control Servo Torque

```json
{
  "type": "TorqueCtrl",
  "id": 1,
  "enable": true
}
```

Parameters:
- `id`: Servo ID (1-6)
- `enable`: Enable (true) or disable (false) torque

#### ServoParams - Set Servo Parameters

```json
{
  "type": "ServoParams",
  "id": 1,
  "minAngle": 0,
  "maxAngle": 180,
  "centerOffset": 5
}
```

Parameters:
- `id`: Servo ID (1-6)
- `minAngle`: Minimum allowed angle
- `maxAngle`: Maximum allowed angle
- `centerOffset`: Center position offset

### 2. Coordinate Control Commands

Commands for controlling the end-effector position:

#### CoordCtrl - Control End-Effector Position

```json
{
  "type": "CoordCtrl",
  "x": 150,
  "y": 0,
  "z": 100,
  "roll": 0,
  "pitch": 90,
  "yaw": 0,
  "speed": 50
}
```

Parameters:
- `x`, `y`, `z`: Cartesian coordinates in mm
- `roll`, `pitch`, `yaw`: Orientation in degrees
- `speed`: Movement speed (1-100)

#### LinearMove - Linear Movement in Cartesian Space

```json
{
  "type": "LinearMove",
  "x": 150,
  "y": 0,
  "z": 100,
  "speed": 50,
  "steps": 10
}
```

Parameters:
- `x`, `y`, `z`: Target coordinates in mm
- `speed`: Movement speed (1-100)
- `steps`: Number of intermediate points for smooth motion

### 3. Sequence Control Commands

Commands for managing motion sequences:

#### SavePos - Save Current Position

```json
{
  "type": "SavePos",
  "name": "position1"
}
```

Parameters:
- `name`: Name to identify the saved position

#### LoadPos - Move to Saved Position

```json
{
  "type": "LoadPos",
  "name": "position1",
  "speed": 50
}
```

Parameters:
- `name`: Name of the saved position
- `speed`: Movement speed (1-100)

#### SaveSeq - Save Sequence

```json
{
  "type": "SaveSeq",
  "name": "sequence1",
  "positions": ["position1", "position2", "position3"],
  "delays": [1000, 1000]
}
```

Parameters:
- `name`: Name to identify the sequence
- `positions`: Array of saved position names
- `delays`: Array of delays between positions (in ms)

#### RunSeq - Run Saved Sequence

```json
{
  "type": "RunSeq",
  "name": "sequence1",
  "loop": false,
  "speed": 50
}
```

Parameters:
- `name`: Name of the saved sequence
- `loop`: Whether to loop the sequence
- `speed`: Movement speed (1-100)

### 4. System Control Commands

Commands for system-level control:

#### Reset - Reset the Arm

```json
{
  "type": "Reset"
}
```

Resets the arm to its default position.

#### GetStatus - Get System Status

```json
{
  "type": "GetStatus"
}
```

Returns the current status of the arm, including joint angles and end-effector position.

#### SetMode - Set Operation Mode

```json
{
  "type": "SetMode",
  "mode": "normal"
}
```

Parameters:
- `mode`: Operation mode ("normal", "learning", "follower", "leader")

### 5. Advanced Control Commands

Commands for specialized control features:

#### DEFA - Dynamic External Force Adaptive Control

```json
{
  "type": "DEFA",
  "enable": true,
  "sensitivity": 50
}
```

Parameters:
- `enable`: Enable or disable DEFA mode
- `sensitivity`: Force sensitivity (1-100)

#### LEDCtrl - Control LED Lighting

```json
{
  "type": "LEDCtrl",
  "brightness": 80,
  "pattern": "solid"
}
```

Parameters:
- `brightness`: LED brightness (0-100)
- `pattern`: Light pattern ("solid", "blink", "pulse", "off")

### 6. Compound Commands

Commands for executing multiple operations:

#### MultiCmd - Execute Multiple Commands

```json
{
  "type": "MultiCmd",
  "cmds": [
    {"type": "AngleCtrl", "id": 1, "angle": 90, "speed": 50},
    {"type": "Delay", "time": 1000},
    {"type": "AngleCtrl", "id": 2, "angle": 45, "speed": 50}
  ]
}
```

Parameters:
- `cmds`: Array of command objects to execute in sequence

#### Delay - Add Delay Between Commands

```json
{
  "type": "Delay",
  "time": 1000
}
```

Parameters:
- `time`: Delay time in milliseconds

## Transmission Methods

The JSON Command System supports multiple methods for sending commands to the RoArm-M3 Pro:

### 1. Web Interface

Commands can be entered directly into the JSON command text box in the web interface:

1. Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678)
2. Navigate to 192.168.4.1 in a web browser
3. Enter the JSON command in the command input box
4. Click "Send" to execute the command

### 2. HTTP Requests

Commands can be sent via HTTP POST requests:

```python
import requests
import json

# Define the command
command = {
    "type": "AngleCtrl",
    "id": 1,
    "angle": 90,
    "speed": 50
}

# Send the command
url = "http://192.168.4.1/cmd"
headers = {"Content-Type": "application/json"}
response = requests.post(url, data=json.dumps(command), headers=headers)

# Process the response
result = response.json()
print(result)
```

### 3. Serial Communication

Commands can be sent via serial communication:

```python
import serial
import json

# Define the command
command = {
    "type": "AngleCtrl",
    "id": 1,
    "angle": 90,
    "speed": 50
}

# Configure serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Send the command
ser.write((json.dumps(command) + '\n').encode())

# Read the response
response = ser.readline().decode().strip()
result = json.loads(response)
print(result)
```

## Error Handling

The JSON Command System includes comprehensive error handling:

### Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": 101,
    "message": "Invalid servo ID"
  }
}
```

### Common Error Codes

| Code | Description |
|------|-------------|
| 100  | Invalid command type |
| 101  | Invalid servo ID |
| 102  | Invalid angle value |
| 103  | Invalid speed value |
| 104  | Position unreachable |
| 105  | Sequence not found |
| 106  | Communication timeout |
| 107  | JSON parsing error |
| 108  | Missing required parameter |
| 109  | System busy |

### Error Handling Best Practices

1. **Validate Commands**: Check command structure and parameter values before sending
2. **Handle Responses**: Always check the "status" field in responses
3. **Implement Retry Logic**: Retry failed commands with appropriate backoff
4. **Log Errors**: Maintain logs of command errors for troubleshooting
5. **Graceful Degradation**: Implement fallback behavior for critical operations

## Advanced Usage

### 1. Leader-Follower Mode

The JSON Command System supports leader-follower mode configuration:

```json
{
  "type": "SetMode",
  "mode": "leader"
}
```

```json
{
  "type": "SetMode",
  "mode": "follower",
  "leaderMAC": "AA:BB:CC:DD:EE:FF"
}
```

### 2. Custom Sequences

Creating and running custom sequences:

```json
// Save multiple positions
{"type": "AngleCtrl", "id": 1, "angle": 90, "speed": 50}
{"type": "SavePos", "name": "pos1"}
{"type": "AngleCtrl", "id": 1, "angle": 45, "speed": 50}
{"type": "SavePos", "name": "pos2"}

// Create a sequence
{
  "type": "SaveSeq",
  "name": "my_sequence",
  "positions": ["pos1", "pos2", "pos1"],
  "delays": [1000, 1000]
}

// Run the sequence
{
  "type": "RunSeq",
  "name": "my_sequence",
  "loop": true,
  "speed": 70
}
```

### 3. Coordinate Transformations

Working with different coordinate systems:

```json
// Set coordinate system origin
{
  "type": "SetCoordOrigin",
  "x": 100,
  "y": 0,
  "z": 50
}

// Move relative to current position
{
  "type": "RelativeMove",
  "dx": 10,
  "dy": 0,
  "dz": -5,
  "speed": 50
}
```

### 4. Event Triggers

Setting up event-based actions:

```json
// Configure position-based trigger
{
  "type": "SetTrigger",
  "condition": {
    "type": "position",
    "id": 1,
    "angle": 90,
    "tolerance": 5
  },
  "action": {
    "type": "AngleCtrl",
    "id": 2,
    "angle": 45,
    "speed": 50
  }
}
```

## Integration Examples

### Python Integration

```python
import requests
import json
import time

class RoArmController:
    def __init__(self, ip="192.168.4.1"):
        self.base_url = f"http://{ip}/cmd"
        self.headers = {"Content-Type": "application/json"}
    
    def send_command(self, command):
        response = requests.post(
            self.base_url, 
            data=json.dumps(command), 
            headers=self.headers
        )
        return response.json()
    
    def move_joint(self, joint_id, angle, speed=50):
        command = {
            "type": "AngleCtrl",
            "id": joint_id,
            "angle": angle,
            "speed": speed
        }
        return self.send_command(command)
    
    def move_to_position(self, x, y, z, speed=50):
        command = {
            "type": "CoordCtrl",
            "x": x,
            "y": y,
            "z": z,
            "speed": speed
        }
        return self.send_command(command)
    
    def run_sequence(self, sequence_name, loop=False, speed=50):
        command = {
            "type": "RunSeq",
            "name": sequence_name,
            "loop": loop,
            "speed": speed
        }
        return self.send_command(command)
    
    def get_status(self):
        command = {"type": "GetStatus"}
        return self.send_command(command)

# Example usage
arm = RoArmController()
arm.move_joint(1, 90)
time.sleep(1)
arm.move_to_position(150, 0, 100)
status = arm.get_status()
print(f"Current position: {status['data']['position']}")
```

### JavaScript Integration

```javascript
class RoArmController {
    constructor(ip = "192.168.4.1") {
        this.baseUrl = `http://${ip}/cmd`;
    }
    
    async sendCommand(command) {
        const response = await fetch(this.baseUrl, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(command)
        });
        return await response.json();
    }
    
    async moveJoint(jointId, angle, speed = 50) {
        const command = {
            type: "AngleCtrl",
            id: jointId,
            angle: angle,
            speed: speed
        };
        return await this.sendCommand(command);
    }
    
    async moveToPosition(x, y, z, speed = 50) {
        const command = {
            type: "CoordCtrl",
            x: x,
            y: y,
            z: z,
            speed: speed
        };
        return await this.sendCommand(command);
    }
    
    async runSequence(sequenceName, loop = false, speed = 50) {
        const command = {
            type: "RunSeq",
            name: sequenceName,
            loop: loop,
            speed: speed
        };
        return await this.sendCommand(command);
    }
    
    async getStatus() {
        const command = {type: "GetStatus"};
        return await this.sendCommand(command);
    }
}

// Example usage
async function controlArm() {
    const arm = new RoArmController();
    await arm.moveJoint(1, 90);
    setTimeout(async () => {
        await arm.moveToPosition(150, 0, 100);
        const status = await arm.getStatus();
        console.log(`Current position: ${JSON.stringify(status.data.position)}`);
    }, 1000);
}

controlArm();
```

## Available Resources

### Documentation
- [RoArm-M3 User Manual](https://www.waveshare.com/wiki/RoArm-M3)
- [JSON Command Reference](https://www.waveshare.com/wiki/RoArm-M3)
- [API Documentation](https://www.waveshare.com/wiki/RoArm-M3)

### Software
- [Python SDK](https://www.waveshare.com/wiki/RoArm-M3)
- [Example Scripts](https://www.waveshare.com/wiki/RoArm-M3)
- [Command Testing Tool](https://www.waveshare.com/wiki/RoArm-M3)

### Related Components
- [ESP32-WROOM-32 Main Control Module](https://www.waveshare.com/wiki/ESP32-DevKitC)
- [Web Interface](https://www.waveshare.com/wiki/RoArm-M3)
- [Python API](https://www.waveshare.com/wiki/RoArm-M3)

## Troubleshooting

### Common Issues

1. **Command Not Executing**
   - **Symptom**: Command sent but no arm movement
   - **Possible Causes**:
     - JSON syntax error
     - Invalid parameter values
     - Arm in error state
     - Communication issue
   - **Solutions**:
     - Validate JSON syntax
     - Check parameter values against allowed ranges
     - Reset the arm
     - Verify network/serial connection

2. **Unexpected Movement**
   - **Symptom**: Arm moves differently than expected
   - **Possible Causes**:
     - Incorrect parameter values
     - Coordinate system misunderstanding
     - Physical limitations
   - **Solutions**:
     - Double-check command parameters
     - Verify coordinate system understanding
     - Check for mechanical obstructions
     - Start with slower speeds for testing

3. **Communication Errors**
   - **Symptom**: Cannot send commands or receive responses
   - **Possible Causes**:
     - Network connectivity issues
     - Serial connection problems
     - System overload
   - **Solutions**:
     - Check WiFi connection
     - Verify serial port settings
     - Reduce command frequency
     - Restart the arm and control device

4. **JSON Parsing Errors**
   - **Symptom**: Error response with parsing error
   - **Possible Causes**:
     - Malformed JSON
     - Missing quotes around string values
     - Trailing commas
     - Invalid characters
   - **Solutions**:
     - Use a JSON validator
     - Check for proper quoting of strings
     - Remove trailing commas
     - Escape special characters properly

## Best Practices

1. **Command Structure**
   - Follow the exact JSON structure for each command type
   - Include all required parameters
   - Use appropriate data types for parameters
   - Keep commands concise and focused

2. **Error Handling**
   - Always check response status
   - Implement appropriate error handling
   - Use try-catch blocks for JSON parsing
   - Log errors for troubleshooting

3. **Performance Optimization**
   - Batch related commands using MultiCmd
   - Use appropriate speeds for different operations
   - Implement rate limiting for command sequences
   - Consider arm dynamics when planning movements

4. **Security Considerations**
   - Restrict network access to the arm
   - Validate all input before sending commands
   - Implement authentication if exposed to public networks
   - Consider encryption for sensitive applications

## Source Attribution

This documentation on the JSON Command System is based on information from the following sources:

1. **Waveshare Wiki**: The core command structure, basic format, and command types are documented on the [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3#Advanced_Use). The wiki provides essential information about:
   - Basic JSON command format and structure
   - Command types and their parameters
   - Transmission methods (Web interface, HTTP, Serial)
   - Example commands for basic operations

2. **Enhanced Documentation**: This document expands upon the Waveshare documentation with:
   - Comprehensive categorization of all command types
   - Detailed parameter descriptions and valid ranges
   - Extended error handling information and error codes
   - Advanced usage examples and integration patterns
   - Troubleshooting guidance and best practices
   - Code examples in multiple programming languages

The goal of this enhanced documentation is to provide a more comprehensive and user-friendly reference for the JSON Command System, making it easier to understand and utilize the full capabilities of the RoArm-M3 Pro robotic arm.

## Configuration Persistence Commands

The following commands are used to save configurations persistently so they remain after power cycling:

| Command | T Value | Description | Example |
|---------|---------|-------------|---------|
| CMD_WIFI_CONFIG_CREATE_BY_STATUS | 406 | Creates a wifiConfig.json file from current settings | `{"T":406}` |
| CMD_WIFI_CONFIG_CREATE_BY_INPUT | 407 | Creates a wifiConfig.json file from provided parameters | `{"T":407,"mode":3,"ap_ssid":"RoArm-M3","ap_password":"12345678","sta_ssid":"YourWifi","sta_password":"YourPassword"}` |
| CMD_WIFI_ON_BOOT | 401 | Sets the WiFi mode on boot (0=OFF, 1=AP, 2=STA, 3=AP+STA) | `{"T":401,"cmd":3}` |
| CMD_NVS_CLEAR | 604 | Clears the NVS (Non-Volatile Storage) | `{"T":604}` |
| CMD_REBOOT | 600 | Reboots the device | `{"T":600}` |

**Important**: After changing settings, you must explicitly save them using `{"T":406}` or `{"T":407,...}` for the changes to persist after power cycling.

## Conclusion

The JSON Command System provides a powerful and flexible interface for controlling the RoArm-M3 Pro robotic arm. By using standardized JSON commands, it enables seamless integration with various programming languages and control systems. Whether controlling the arm through the web interface, HTTP requests, or serial communication, the JSON Command System offers a consistent and comprehensive method for accessing all arm functions and features. Understanding the command structure, parameter requirements, and best practices outlined in this documentation will enable effective utilization of the RoArm-M3 Pro's capabilities for a wide range of applications.
