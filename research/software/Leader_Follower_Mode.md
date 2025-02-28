# Leader-Follower Mode for RoArm-M3 Pro

## Overview

Leader-Follower Mode (also known as Imitation Mode) is an advanced feature of the RoArm-M3 Pro robotic arm that enables one arm to act as a "Leader" and transmit its joint positions to other RoArm-M3 devices, which then act as "Followers" and replicate the Leader's movements in real-time. This capability is implemented using ESP-NOW, a peer-to-peer communication protocol developed by Espressif for ESP32 microcontrollers, which allows for low-latency, connectionless communication between devices.

This feature enables a wide range of applications, including synchronized multi-arm operations, teaching by demonstration, and remote control scenarios. By allowing one arm to control multiple others, complex choreographed movements can be achieved without the need to program each arm individually.

## Key Features

- **Real-time Movement Mirroring**: Follower arms replicate the Leader's movements with minimal latency
- **Wireless Communication**: Uses ESP-NOW protocol for direct device-to-device communication
- **Multiple Follower Support**: One Leader can control multiple Follower arms simultaneously
- **No Router Required**: Direct communication without the need for a WiFi router
- **Low Latency**: Designed for real-time control with minimal delay
- **Simple Configuration**: Easy setup through the web interface or JSON commands
- **Automatic Reconnection**: Followers automatically reconnect to the Leader if connection is lost
- **Range**: Typical effective range of 30-50 meters in open areas

## Technical Implementation

### ESP-NOW Protocol

The Leader-Follower Mode is built on ESP-NOW, a connectionless communication protocol that offers several advantages for robotic control:

1. **Low Overhead**: Minimal protocol overhead results in faster transmission times
2. **No Connection Setup**: Devices can communicate without establishing a formal connection
3. **Low Power Consumption**: Efficient protocol designed for battery-powered devices
4. **Peer-to-Peer Architecture**: Direct communication between devices without intermediaries
5. **MAC Address Based**: Uses MAC addresses for device identification and targeting

### Communication Architecture

The Leader-Follower system uses a simple master-slave architecture:

1. **Leader (Master)**:
   - Broadcasts its joint positions to registered Follower devices
   - Transmits data whenever joint positions change
   - Can operate in normal control modes while broadcasting

2. **Follower (Slave)**:
   - Listens for broadcasts from the Leader
   - Updates its joint positions based on received data
   - Ignores other control inputs while in Follower mode

### Data Transmission

The data transmitted between devices includes:

1. **Joint Angles**: Current position of each servo in the Leader arm
2. **Movement Speed**: Speed parameters for the movement
3. **Control Flags**: Additional control information
4. **Checksum**: Data validation to ensure integrity

## Setup and Configuration

### Hardware Requirements

To use the Leader-Follower Mode, you will need:

1. **Two or more RoArm-M3 Pro robotic arms**
2. **Power supply for each arm** (12V 5A power adapter or 3S lithium battery)
3. **Clear line of sight** between arms for optimal wireless communication

### Leader Configuration

To configure a RoArm-M3 Pro as the Leader:

#### Using the Web Interface

1. Power on the RoArm-M3 Pro
2. Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678)
3. Navigate to 192.168.4.1 in a web browser
4. Go to the Advanced Settings section
5. Select "ESP-NOW Control" from the menu
6. Choose "Leader Mode" from the dropdown
7. Click "Apply" to save the settings
8. The arm will restart and enter Leader mode

#### Using JSON Commands

To configure the arm as a Leader using JSON commands:

```json
{
  "type": "ESPNowConfig",
  "mode": "leader"
}
```

Send this command through either HTTP or Serial communication as described in the [JSON Command System documentation](./JSON_Command_System.md).

### Follower Configuration

To configure a RoArm-M3 Pro as a Follower:

#### Using the Web Interface

1. Power on the RoArm-M3 Pro
2. Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678)
3. Navigate to 192.168.4.1 in a web browser
4. Go to the Advanced Settings section
5. Select "ESP-NOW Control" from the menu
6. Choose "Follower Mode" from the dropdown
7. Enter the MAC address of the Leader arm (displayed on the Leader's OLED screen)
8. Click "Apply" to save the settings
9. The arm will restart and enter Follower mode

#### Using JSON Commands

To configure the arm as a Follower using JSON commands:

```json
{
  "type": "ESPNowConfig",
  "mode": "follower",
  "leaderMac": "XX:XX:XX:XX:XX:XX"  // Replace with the Leader's MAC address
}
```

Send this command through either HTTP or Serial communication.

### Finding the MAC Address

The MAC address of each RoArm-M3 Pro is displayed on the OLED screen after power-on. It is shown on the third line of the display. Alternatively, you can retrieve it using the following JSON command:

```json
{
  "type": "GetStatus"
}
```

The response will include the MAC address in the "mac" field.

## Usage

### Basic Operation

Once the Leader and Follower arms are configured:

1. Power on the Leader arm first
2. Power on the Follower arm(s)
3. Wait for the Follower(s) to connect to the Leader (indicated by a status message on the OLED display)
4. Control the Leader arm using any of the standard control methods:
   - Web interface
   - JSON commands via HTTP
   - JSON commands via Serial
   - Physical manipulation (when using DEFA mode)
5. The Follower arm(s) will automatically mimic the movements of the Leader

### Teaching by Demonstration

One powerful application of Leader-Follower Mode is teaching by demonstration:

1. Configure one arm as the Leader and others as Followers
2. On the Leader arm, disable torque using the "Torque OFF" command
3. Manually position the Leader arm through a sequence of movements
4. The Follower arms will replicate these movements in real-time
5. To record the sequence for later playback, use a computer to capture the joint positions at key points

### Synchronized Operations

For performing synchronized operations with multiple arms:

1. Configure one arm as the Leader and others as Followers
2. Program or control the Leader arm to perform the desired sequence
3. All Follower arms will execute the same sequence simultaneously
4. This is particularly useful for production lines or educational demonstrations

## Performance Considerations

### Range and Interference

The effective range of ESP-NOW communication depends on several factors:

1. **Physical Environment**: Walls, metal objects, and other obstacles can reduce range
2. **Interference**: Other 2.4GHz devices can cause interference
3. **Antenna Orientation**: The orientation of the antennas affects signal strength
4. **Power Level**: Higher power levels increase range but consume more battery

For optimal performance:

- Maintain clear line of sight between arms when possible
- Keep arms within 30 meters of each other
- Minimize other 2.4GHz wireless traffic in the area
- Position antennas vertically for best signal propagation

### Latency

The Leader-Follower Mode is designed for low-latency operation, but some delay is inevitable:

1. **Transmission Delay**: Typically 5-15ms for ESP-NOW communication
2. **Processing Delay**: Time required to process received data (5-10ms)
3. **Mechanical Delay**: Time required for servos to respond to commands (varies by load)

Total system latency is typically in the range of 20-50ms, which is suitable for most applications but may be noticeable in high-speed operations.

### Battery Life

When operating in Leader-Follower Mode, power consumption increases due to the additional wireless communication:

1. **Leader Mode**: Approximately 10-15% higher power consumption than standard operation
2. **Follower Mode**: Approximately 5-10% higher power consumption than standard operation

For battery-powered operation, consider:

- Using higher capacity batteries
- Implementing power-saving strategies when not actively using the arms
- Monitoring battery levels to prevent unexpected shutdowns

## Troubleshooting

### Common Issues

1. **Follower Not Connecting**
   - **Symptom**: Follower arm does not mirror the Leader's movements
   - **Possible Causes**: Incorrect MAC address, out of range, interference
   - **Solutions**: 
     - Verify the Leader's MAC address is correctly entered
     - Move the arms closer together
     - Restart both arms
     - Check for sources of interference

2. **Erratic Movement**
   - **Symptom**: Follower arm moves erratically or inconsistently
   - **Possible Causes**: Packet loss, interference, mechanical limitations
   - **Solutions**: 
     - Reduce distance between arms
     - Move away from sources of interference
     - Slow down the Leader's movements
     - Check for mechanical issues in the Follower arm

3. **Delayed Response**
   - **Symptom**: Significant delay between Leader and Follower movements
   - **Possible Causes**: High network traffic, processing limitations, mechanical issues
   - **Solutions**: 
     - Reduce distance between arms
     - Minimize other wireless traffic
     - Slow down the Leader's movements
     - Check for mechanical binding in the Follower arm

4. **Connection Lost During Operation**
   - **Symptom**: Follower stops responding to Leader
   - **Possible Causes**: Range exceeded, interference, power issues
   - **Solutions**: 
     - Move arms closer together
     - Check power supplies
     - Restart both arms
     - Reconfigure the connection

### Resetting to Default Mode

If you need to reset an arm from Leader or Follower mode to standard operation:

#### Using the Web Interface

1. Connect to the arm's WiFi
2. Navigate to 192.168.4.1 in a web browser
3. Go to the Advanced Settings section
4. Select "ESP-NOW Control" from the menu
5. Choose "Disabled" from the dropdown
6. Click "Apply" to save the settings

#### Using JSON Commands

```json
{
  "type": "ESPNowConfig",
  "mode": "disabled"
}
```

## Advanced Configuration

### Multiple Leader-Follower Groups

It is possible to have multiple Leader-Follower groups operating in the same area:

1. Each group should use a different Leader
2. Followers are paired specifically with their designated Leader
3. Each group operates independently of the others

This allows for complex choreographed movements with multiple synchronized groups.

### Mixing Control Methods

While in Leader-Follower Mode:

1. **Leader Arm**: Can be controlled using any standard method
2. **Follower Arms**: Will only respond to commands from the Leader, ignoring other inputs

This allows for flexible control strategies where the Leader can be controlled manually, programmatically, or through AI systems, while the Followers automatically replicate its movements.

## Integration with Other Control Features

The Leader-Follower Mode integrates with other RoArm-M3 Pro control features:

### 1. AngleCtrl (Servo Angle Control)

- When the Leader receives AngleCtrl commands, the movements are transmitted to Followers
- This allows for precise programmatic control of multiple arms

### 2. COORDCTRL (End Point Coordinate Control)

- When the Leader receives COORDCTRL commands, the resulting joint positions are transmitted to Followers
- This enables synchronized end-effector positioning across multiple arms

### 3. DEFA (Dynamic External Force Adaptive Control)

- When DEFA is enabled on the Leader, manual positioning is transmitted to Followers
- This creates a powerful teaching-by-demonstration capability

## Available Resources

### Documentation
- [RoArm-M3 Wiki Page](https://www.waveshare.com/wiki/RoArm-M3)
- [ESP-NOW Protocol Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [JSON Command System Documentation](./JSON_Command_System.md)
- [Python API Documentation](./Python_API.md)

### Hardware Resources
- [ESP32-WROOM-32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)
- [ST3235 Metal Shell Servo Datasheet](https://www.waveshare.com/st3235-servo.htm)
- [General Driver Board for Robots Schematic](https://files.waveshare.com/wiki/common/General_Driver_for_Robots_SCH.pdf)

### Software Resources
- [RoArm-M3 Python Demo](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip)
- [RoArm-M3 Open Source Program](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Open_Source_Program.zip)
- [ESP-NOW Example Code](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/ESPNow)

## Example Code

### Python Example for Controlling a Leader Arm

This example demonstrates how to control a Leader arm, which will then transmit its movements to Follower arms:

```python
import requests
import json
import time

def configure_leader_mode(ip_address):
    """Configure the arm as a Leader for ESP-NOW communication."""
    cmd = {
        "type": "ESPNowConfig",
        "mode": "leader"
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Leader configuration response: {response.text}")
    # Allow time for the arm to restart and enter Leader mode
    time.sleep(5)

def move_leader_arm(ip_address, sequence):
    """Move the Leader arm through a sequence of positions."""
    for position in sequence:
        cmd = {
            "type": "AngleCtrl",
            "id": position["joint"],
            "angle": position["angle"],
            "speed": position["speed"]
        }
        url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
        response = requests.get(url)
        print(f"Movement command response: {response.text}")
        # Allow time for the movement to complete
        time.sleep(position["duration"])

# Example usage
if __name__ == "__main__":
    # IP address of the Leader arm
    leader_ip = "192.168.4.1"
    
    # Configure the arm as a Leader
    configure_leader_mode(leader_ip)
    
    # Define a sequence of movements
    movement_sequence = [
        {"joint": 1, "angle": 1.57, "speed": 50, "duration": 2},  # Base joint to 90 degrees
        {"joint": 2, "angle": 0.78, "speed": 30, "duration": 2},  # Shoulder joint to 45 degrees
        {"joint": 3, "angle": 1.57, "speed": 40, "duration": 2},  # Elbow joint to 90 degrees
        {"joint": 4, "angle": 0.0, "speed": 50, "duration": 2},   # Wrist joint 1 to 0 degrees
        {"joint": 5, "angle": 0.0, "speed": 50, "duration": 2},   # Wrist joint 2 to 0 degrees
        {"joint": 6, "angle": 2.0, "speed": 50, "duration": 2},   # Gripper partially open
    ]
    
    # Execute the movement sequence on the Leader arm
    # (Follower arms will automatically mimic these movements)
    move_leader_arm(leader_ip, movement_sequence)
```

### Python Example for Configuring a Follower Arm

This example demonstrates how to configure an arm as a Follower:

```python
import requests
import json
import time

def configure_follower_mode(ip_address, leader_mac):
    """Configure the arm as a Follower for ESP-NOW communication."""
    cmd = {
        "type": "ESPNowConfig",
        "mode": "follower",
        "leaderMac": leader_mac
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Follower configuration response: {response.text}")
    # Allow time for the arm to restart and enter Follower mode
    time.sleep(5)

def get_mac_address(ip_address):
    """Get the MAC address of the arm."""
    cmd = {
        "type": "GetStatus"
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    status = json.loads(response.text)
    return status.get("mac", "")

# Example usage
if __name__ == "__main__":
    # IP address of the Follower arm
    follower_ip = "192.168.4.1"
    
    # IP address of the Leader arm (to get its MAC address)
    leader_ip = "192.168.4.1"  # Note: You would need to connect to each arm separately
    
    # Get the Leader's MAC address
    leader_mac = get_mac_address(leader_ip)
    print(f"Leader MAC address: {leader_mac}")
    
    # Configure the arm as a Follower
    configure_follower_mode(follower_ip, leader_mac)
    
    print("Follower arm configured successfully. It will now mimic the Leader's movements.")
```

## Conclusion

The Leader-Follower Mode is a powerful feature of the RoArm-M3 Pro that enables synchronized movement across multiple robotic arms. By leveraging the ESP-NOW protocol, this mode provides low-latency, wireless control without the need for complex networking infrastructure. This capability opens up a wide range of applications, from educational demonstrations to industrial automation, where coordinated movement of multiple arms is required.

Understanding how to properly configure and use this mode, along with its integration with other control features, allows users to fully exploit the potential of the RoArm-M3 Pro in multi-arm scenarios. While there are some limitations in terms of range and latency, the system provides a robust and flexible solution for most applications requiring synchronized robotic arm movement.
