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

### ESP-NOW Configuration

The RoArm-M3 Pro uses specific JSON commands for ESP-NOW configuration. These commands are accessible through the web interface at 192.168.4.1 after connecting to the arm's WiFi (SSID: RoArm-M3, Password: 12345678).

#### Getting the MAC Address

Before setting up Leader-Follower mode, you need to know the MAC address of each arm:

```json
{"T":302}  // CMD_GET_MAC_ADDRESS
```

This command returns the MAC address of the arm, which you'll need for configuring the Leader-Follower relationship.

#### Setting Up a Leader

To configure an arm as a Leader, use the ESP-NOW configuration command with the appropriate mode parameter:

```json
{"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}  // CMD_ESP_NOW_CONFIG for F-LEADER-B (Broadcast) mode
```

or

```json
{"T":301,"mode":2,"dev":0,"cmd":0,"megs":0}  // CMD_ESP_NOW_CONFIG for F-LEADER-S (Single Follower) mode
```

The `mode` parameter determines the role:
- `mode=1`: F-LEADER-B (Flow-Leader Broadcast) mode - for controlling multiple Followers
- `mode=2`: F-LEADER-S (Flow-Leader Single) mode - for controlling a single Follower
- `mode=0`: Disabled (normal operation)

Once configured as a Leader, you need to enable broadcasting to Followers:

```json
{"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}  // CMD_BROADCAST_FOLLOWER
```

The `mode` parameter controls the broadcast state:
- `mode=1`: Enable broadcasting
- `mode=0`: Disable broadcasting

#### Setting Up a Follower

To configure an arm as a Follower, first add the Leader's MAC address to the Follower's list:

```json
{"T":303,"mac":"XX:XX:XX:XX:XX:XX"}  // CMD_ESP_NOW_ADD_FOLLOWER
```

Replace `XX:XX:XX:XX:XX:XX` with the actual MAC address of the Leader arm.

Then, enable Follower mode:

```json
{"T":301,"mode":3,"dev":0,"cmd":0,"megs":0}  // CMD_ESP_NOW_CONFIG for FOLLOWER mode
```

#### Security Considerations

The ESP-NOW protocol used by the RoArm-M3 Pro relies on MAC addresses for device identification, which presents some security considerations:

1. **MAC Address Authentication**: The system uses MAC addresses for authentication, which can potentially be spoofed
2. **No Encryption**: By default, ESP-NOW does not encrypt the data transmitted between devices
3. **Physical Access Control**: Ensure that only authorized personnel have physical access to the arms
4. **Isolated Network**: Consider operating the arms on an isolated network to prevent unauthorized access

For enhanced security:
- Regularly update the firmware to the latest version
- Use the arms in a controlled environment
- Consider implementing additional authentication mechanisms if using in sensitive applications

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

### Controlling Followers

Once the Leader-Follower configuration is set up, there are two ways to control the Followers:

#### Controlling All Followers

To send commands to all connected Followers simultaneously:

```json
{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":"hello!"}  // CMD_ESP_NOW_MANY_CTRL
```

This command sends joint position data to all connected Followers, where:
- `b`, `s`, `e`, `t`, `r`, `h`: Joint positions for base, shoulder, elbow, wrist, roll, and hand
- `cmd`: Additional control command
- `megs`: Optional message

#### Controlling a Specific Follower

To control a specific Follower:

```json
{"T":306,"mac":"XX:XX:XX:XX:XX:XX","dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":"hello!"}  // CMD_ESP_NOW_SINGLE
```

Replace `XX:XX:XX:XX:XX:XX` with the MAC address of the specific Follower arm.

### Resetting to Default Mode

If you need to reset an arm from Leader or Follower mode to standard operation:

```json
{"T":301,"mode":0,"dev":0,"cmd":0,"megs":0}  // CMD_ESP_NOW_CONFIG with mode=0 for disabled
```

### ESP-NOW Command Reference

| Command | JSON Format | Description |
|---------|-------------|-------------|
| Get MAC Address | `{"T":302}` | Retrieves the MAC address of the arm |
| ESP-NOW Config | `{"T":301,"mode":X,"dev":0,"cmd":0,"megs":0}` | Configures ESP-NOW mode (0=Disabled, 1=F-LEADER-B (Broadcast), 2=F-LEADER-S (Single), 3=FOLLOWER) |
| Add Follower | `{"T":303,"mac":"XX:XX:XX:XX:XX:XX"}` | Adds a Follower to the Leader's list |
| Remove Follower | `{"T":304,"mac":"XX:XX:XX:XX:XX:XX"}` | Removes a Follower from the Leader's list |
| Broadcast Control | `{"T":300,"mode":X,"mac":"FF:FF:FF:FF:FF:FF"}` | Enables/disables broadcasting (0=Disabled, 1=Enabled) |
| Control All Followers | `{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":""}` | Controls all connected Followers |
| Control Specific Follower | `{"T":306,"mac":"XX:XX:XX:XX:XX:XX","dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":""}` | Controls a specific Follower |

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
- [General Driver Board for Robots Schematic](https://files.waveshare.com/wiki/common/General_Driver_for_Robots_SCH.pdf) ([Local Copy](../hardware/main_control/ESP32-WROOM-32/datasheets/General_Driver_for_Robots_SCH.pdf))

### Software Resources
- [RoArm-M3 Python Demo](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip) ([Local Copy](../hardware/misc/misc/drivers/RoArm-M3_Python.zip))
- [RoArm-M3 Open Source Program](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Open_Source_Program.zip) ([Local Copy](../hardware/misc/misc/drivers/RoArm-M3_Open_Source_Program.zip))
- [ESP-NOW Example Code](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/ESPNow)

## Example Code

### Python Example for Leader-Follower Configuration

This example demonstrates how to configure and use the Leader-Follower Mode with the correct JSON commands:

```python
import requests
import json
import time

def get_mac_address(ip_address):
    """Get the MAC address of the arm."""
    cmd = {"T":302}  # CMD_GET_MAC_ADDRESS
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    data = json.loads(response.text)
    mac_address = data.get("mac", "")
    print(f"MAC Address: {mac_address}")
    return mac_address

def configure_leader_mode(ip_address, mode=1):
    """Configure the arm as a Leader for ESP-NOW communication.
    
    Args:
        ip_address: IP address of the arm
        mode: 1 for F-LEADER-B (Broadcast) mode, 2 for F-LEADER-S (Single) mode
    """
    cmd = {"T":301,"mode":mode,"dev":0,"cmd":0,"megs":0}  # CMD_ESP_NOW_CONFIG with mode=1 for F-LEADER-B or mode=2 for F-LEADER-S
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Leader configuration response: {response.text}")
    
    # Enable broadcasting to followers
    broadcast_cmd = {"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}  # CMD_BROADCAST_FOLLOWER
    url = f"http://{ip_address}/js?json={json.dumps(broadcast_cmd)}"
    response = requests.get(url)
    print(f"Broadcast enable response: {response.text}")
    
    time.sleep(2)  # Allow time for configuration to apply

def add_follower_to_leader(leader_ip, follower_mac):
    """Add a follower to the leader's list."""
    cmd = {"T":303,"mac":follower_mac}  # CMD_ESP_NOW_ADD_FOLLOWER
    url = f"http://{leader_ip}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Add follower response: {response.text}")

def configure_follower_mode(ip_address, leader_mac):
    """Configure the arm as a Follower for ESP-NOW communication."""
    # First add the leader's MAC address
    add_cmd = {"T":303,"mac":leader_mac}  # CMD_ESP_NOW_ADD_FOLLOWER
    url = f"http://{ip_address}/js?json={json.dumps(add_cmd)}"
    response = requests.get(url)
    print(f"Add leader response: {response.text}")
    
    # Then set follower mode
    cmd = {"T":301,"mode":3,"dev":0,"cmd":0,"megs":0}  # CMD_ESP_NOW_CONFIG with mode=3 for Follower
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Follower configuration response: {response.text}")
    
    time.sleep(2)  # Allow time for configuration to apply

def control_all_followers(leader_ip, joint_positions):
    """Control all followers with the specified joint positions."""
    cmd = {
        "T": 305,  # CMD_ESP_NOW_MANY_CTRL
        "dev": 0,
        "b": joint_positions["base"],
        "s": joint_positions["shoulder"],
        "e": joint_positions["elbow"],
        "t": joint_positions["wrist"],
        "r": joint_positions["roll"],
        "h": joint_positions["hand"],
        "cmd": 0,
        "megs": "position update"
    }
    url = f"http://{leader_ip}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Control all followers response: {response.text}")

def control_specific_follower(leader_ip, follower_mac, joint_positions):
    """Control a specific follower with the specified joint positions."""
    cmd = {
        "T": 306,  # CMD_ESP_NOW_SINGLE
        "mac": follower_mac,
        "dev": 0,
        "b": joint_positions["base"],
        "s": joint_positions["shoulder"],
        "e": joint_positions["elbow"],
        "t": joint_positions["wrist"],
        "r": joint_positions["roll"],
        "h": joint_positions["hand"],
        "cmd": 0,
        "megs": "position update"
    }
    url = f"http://{leader_ip}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Control specific follower response: {response.text}")

def move_leader_arm(ip_address, joint_id, angle, speed):
    """Move a specific joint on the Leader arm."""
    cmd = {
        "T": 101,  # CMD_SINGLE_JOINT_CTRL
        "joint": joint_id,
        "rad": angle,
        "spd": speed,
        "acc": 10
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Movement command response: {response.text}")

# Example usage
if __name__ == "__main__":
    # Note: You would need to connect to each arm separately
    leader_ip = "192.168.4.1"
    follower_ip = "192.168.4.1"
    
    # Step 1: Get the MAC addresses
    leader_mac = get_mac_address(leader_ip)
    follower_mac = get_mac_address(follower_ip)
    
    # Step 2: Configure the Leader (using F-LEADER-B mode by default)
    configure_leader_mode(leader_ip, mode=1)  # mode=1 for F-LEADER-B (Broadcast) or mode=2 for F-LEADER-S (Single)
    
    # Step 3: Add the Follower to the Leader's list
    add_follower_to_leader(leader_ip, follower_mac)
    
    # Step 4: Configure the Follower
    configure_follower_mode(follower_ip, leader_mac)
    
    # Step 5: Move the Leader arm (Follower will automatically follow)
    move_leader_arm(leader_ip, 0, 1.57, 50)  # Move base joint to 90 degrees
    time.sleep(2)
    move_leader_arm(leader_ip, 1, 0.78, 30)  # Move shoulder joint to 45 degrees
    time.sleep(2)
    move_leader_arm(leader_ip, 2, 1.57, 40)  # Move elbow joint to 90 degrees
    
    # Step 6: Directly control all followers
    joint_positions = {
        "base": 0.0,
        "shoulder": 0.5,
        "elbow": 1.0,
        "wrist": 0.0,
        "roll": 0.0,
        "hand": 1.57
    }
    control_all_followers(leader_ip, joint_positions)
```

## Conclusion

The Leader-Follower Mode is a powerful feature of the RoArm-M3 Pro that enables synchronized movement across multiple robotic arms. By leveraging the ESP-NOW protocol, this mode provides low-latency, wireless control without the need for complex networking infrastructure. This capability opens up a wide range of applications, from educational demonstrations to industrial automation, where coordinated movement of multiple arms is required.

Understanding how to properly configure and use this mode, along with its integration with other control features, allows users to fully exploit the potential of the RoArm-M3 Pro in multi-arm scenarios. While there are some limitations in terms of range and latency, the system provides a robust and flexible solution for most applications requiring synchronized robotic arm movement.
