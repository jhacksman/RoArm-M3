# Leader-Follower Mode for RoArm-M3 Pro

## Overview

Leader-Follower Mode (also known as Imitation Mode) is an advanced feature of the RoArm-M3 Pro robotic arm that enables one arm to act as a "Leader" and transmit its joint positions to other RoArm-M3 devices, which then act as "Followers" and replicate the Leader's movements in real-time. This capability is implemented using ESP-NOW, a peer-to-peer communication protocol developed by Espressif for ESP32 microcontrollers.

![Leader-Follower Mode Concept](https://www.waveshare.com/w/upload/9/95/500px-RAM3-web1%282%29.png)

## How It Works

The Leader-Follower Mode operates through the following mechanism:

1. **Leader Configuration**: One RoArm-M3 Pro is designated as the Leader
2. **Follower Configuration**: Other RoArm-M3 Pro arms are configured as Followers and paired with the Leader
3. **Real-time Transmission**: The Leader broadcasts its joint positions via ESP-NOW whenever they change
4. **Movement Replication**: Follower arms receive the joint position data and adjust their servos accordingly
5. **Synchronized Movement**: All Follower arms mirror the Leader's movements with minimal latency

## Key Components

The Leader-Follower Mode relies on several key components of the RoArm-M3 Pro:

1. **ESP32-WROOM-32 Microcontroller**: Provides the processing power and wireless capabilities
2. **ESP-NOW Protocol**: Enables low-latency, peer-to-peer communication between devices
3. **ST3235/ST3215 Servo Motors**: High-precision servos that can accurately replicate positions
4. **IPEX Gen 1 WIFI Interface**: Antenna connection for wireless communication
5. **JSON Command System**: Used for configuration and control

## Setup Instructions

### Hardware Requirements

- Two or more RoArm-M3 Pro robotic arms
- Power supply for each arm (12V 5A power adapter or 3S lithium battery)
- Clear line of sight between arms for optimal wireless communication

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
{"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}  // CMD_ESP_NOW_CONFIG
```

The `mode` parameter determines the role:
- `mode=1`: Leader mode
- `mode=0`: Disabled (normal operation)

#### Setting Up a Follower

To configure an arm as a Follower, first add the Leader's MAC address to the Follower's list:

```json
{"T":303,"mac":"XX:XX:XX:XX:XX:XX"}  // CMD_ESP_NOW_ADD_FOLLOWER
```

Replace `XX:XX:XX:XX:XX:XX` with the actual MAC address of the Leader arm.

Then, enable Follower mode:

```json
{"T":301,"mode":2,"dev":0,"cmd":0,"megs":0}  // CMD_ESP_NOW_CONFIG
```

#### Broadcasting to Followers

Once configured, the Leader can broadcast commands to all Followers:

```json
{"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}  // CMD_BROADCAST_FOLLOWER
```

The `mode` parameter controls the broadcast state:
- `mode=1`: Enable broadcasting
- `mode=0`: Disable broadcasting

#### Controlling Multiple Followers

To control multiple Followers simultaneously:

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

#### Removing a Follower

To remove a Follower from the Leader's list:

```json
{"T":304,"mac":"XX:XX:XX:XX:XX:XX"}  // CMD_ESP_NOW_REMOVE_FOLLOWER
```

### Security Considerations

The ESP-NOW protocol used by the RoArm-M3 Pro relies on MAC addresses for device identification, which presents some security considerations:

1. **MAC Address Authentication**: The system uses MAC addresses for authentication, which can potentially be spoofed
2. **No Encryption**: By default, ESP-NOW does not encrypt the data transmitted between devices
3. **Physical Access Control**: Ensure that only authorized personnel have physical access to the arms
4. **Isolated Network**: Consider operating the arms on an isolated network to prevent unauthorized access

For enhanced security:
- Regularly update the firmware to the latest version
- Use the arms in a controlled environment
- Consider implementing additional authentication mechanisms if using in sensitive applications

## Usage Examples

### Basic Operation

1. Power on the Leader arm first
2. Power on the Follower arm(s)
3. Configure the Leader using the ESP-NOW configuration commands
4. Configure the Follower(s) with the Leader's MAC address
5. Enable broadcasting from the Leader
6. Control the Leader arm using any standard method (web interface, JSON commands, etc.)
7. The Follower arm(s) will automatically mimic the movements of the Leader

### Teaching by Demonstration

1. Configure one arm as the Leader and others as Followers using the ESP-NOW commands
2. On the Leader arm, disable torque using the Torque Control command: `{"T":210,"cmd":0}`
3. Manually position the Leader arm through a sequence of movements
4. The Follower arms will replicate these movements in real-time
5. To record the sequence for later playback, use a computer to capture the joint positions

### Synchronized Operations

1. Configure one arm as the Leader and others as Followers using the ESP-NOW commands
2. Program or control the Leader arm to perform the desired sequence
3. All Follower arms will execute the same sequence simultaneously
4. This is particularly useful for production lines or educational demonstrations

## Python Example

The following Python script demonstrates how to configure and use the Leader-Follower Mode with the correct JSON commands:

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

def configure_leader_mode(ip_address):
    """Configure the arm as a Leader for ESP-NOW communication."""
    cmd = {"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}  # CMD_ESP_NOW_CONFIG with mode=1 for Leader
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
    cmd = {"T":301,"mode":2,"dev":0,"cmd":0,"megs":0}  # CMD_ESP_NOW_CONFIG with mode=2 for Follower
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
    
    # Step 2: Configure the Leader
    configure_leader_mode(leader_ip)
    
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

## Troubleshooting

### Common Issues

1. **Follower Not Connecting**
   - Verify the Leader's MAC address is correctly entered in the CMD_ESP_NOW_ADD_FOLLOWER command
   - Ensure the Leader is in mode 1 and the Follower is in mode 2 in the CMD_ESP_NOW_CONFIG command
   - Move the arms closer together
   - Restart both arms
   - Check for sources of interference

2. **Erratic Movement**
   - Reduce distance between arms
   - Move away from sources of interference
   - Slow down the Leader's movements
   - Check for mechanical issues in the Follower arm
   - Verify the broadcasting is enabled with CMD_BROADCAST_FOLLOWER

3. **Connection Lost During Operation**
   - Move arms closer together
   - Check power supplies
   - Restart both arms
   - Reconfigure the connection using the ESP-NOW commands
   - Check if the MAC addresses are still correctly configured

4. **Security Concerns**
   - The ESP-NOW protocol relies on MAC addresses for authentication, which can be spoofed
   - Consider operating in a controlled environment
   - Implement additional security measures for sensitive applications
   - Regularly update firmware to the latest version

## Performance Considerations

- **Range**: Typical effective range of 30-50 meters in open areas
- **Latency**: Total system latency is typically 20-50ms
- **Power Consumption**: Higher than standard operation due to wireless communication
- **Interference**: Other 2.4GHz devices can affect performance
- **Security**: MAC address-based authentication has limitations

## ESP-NOW Command Reference

| Command | JSON Format | Description |
|---------|-------------|-------------|
| Get MAC Address | `{"T":302}` | Retrieves the MAC address of the arm |
| ESP-NOW Config | `{"T":301,"mode":X,"dev":0,"cmd":0,"megs":0}` | Configures ESP-NOW mode (0=Disabled, 1=Leader, 2=Follower) |
| Add Follower | `{"T":303,"mac":"XX:XX:XX:XX:XX:XX"}` | Adds a Follower to the Leader's list |
| Remove Follower | `{"T":304,"mac":"XX:XX:XX:XX:XX:XX"}` | Removes a Follower from the Leader's list |
| Broadcast Control | `{"T":300,"mode":X,"mac":"FF:FF:FF:FF:FF:FF"}` | Enables/disables broadcasting (0=Disabled, 1=Enabled) |
| Control All Followers | `{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":""}` | Controls all connected Followers |
| Control Specific Follower | `{"T":306,"mac":"XX:XX:XX:XX:XX:XX","dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":""}` | Controls a specific Follower |

## Configuration Persistence

To ensure that your Leader-Follower mode settings persist after power cycling:

1. Configure the Leader and Follower modes as described above
2. Save the configuration using the following JSON command:
   ```json
   {"T":406}
   ```
3. Reboot the device to verify that settings persist:
   ```json
   {"T":600}
   ```

For more detailed information on configuration persistence, see the [Configuration Persistence Guide](../research/software/configuration/Configuration_Persistence.md).

## Additional Resources

- [Detailed Leader-Follower Mode Documentation](../research/software/Leader_Follower_Mode.md)
- [ESP-NOW Protocol Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [RoArm-M3 Wiki Page](https://www.waveshare.com/wiki/RoArm-M3)
- [JSON Command System Documentation](../research/software/JSON_Command_System.md)
- [Python API Documentation](../research/software/Python_API.md)
