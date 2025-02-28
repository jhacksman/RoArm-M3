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

### Leader Configuration

To configure a RoArm-M3 Pro as the Leader:

1. Power on the RoArm-M3 Pro
2. Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678)
3. Navigate to 192.168.4.1 in a web browser
4. Go to the Advanced Settings section
5. Select "ESP-NOW Control" from the menu
6. Choose "Leader Mode" from the dropdown
7. Click "Apply" to save the settings
8. The arm will restart and enter Leader mode

Alternatively, use the following JSON command:

```json
{
  "type": "ESPNowConfig",
  "mode": "leader"
}
```

### Follower Configuration

To configure a RoArm-M3 Pro as a Follower:

1. Power on the RoArm-M3 Pro
2. Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678)
3. Navigate to 192.168.4.1 in a web browser
4. Go to the Advanced Settings section
5. Select "ESP-NOW Control" from the menu
6. Choose "Follower Mode" from the dropdown
7. Enter the MAC address of the Leader arm (displayed on the Leader's OLED screen)
8. Click "Apply" to save the settings
9. The arm will restart and enter Follower mode

Alternatively, use the following JSON command:

```json
{
  "type": "ESPNowConfig",
  "mode": "follower",
  "leaderMac": "XX:XX:XX:XX:XX:XX"  // Replace with the Leader's MAC address
}
```

## Usage Examples

### Basic Operation

1. Power on the Leader arm first
2. Power on the Follower arm(s)
3. Wait for the Follower(s) to connect to the Leader (indicated on the OLED display)
4. Control the Leader arm using any standard method (web interface, JSON commands, etc.)
5. The Follower arm(s) will automatically mimic the movements of the Leader

### Teaching by Demonstration

1. Configure one arm as the Leader and others as Followers
2. On the Leader arm, disable torque using the "Torque OFF" command
3. Manually position the Leader arm through a sequence of movements
4. The Follower arms will replicate these movements in real-time
5. To record the sequence for later playback, use a computer to capture the joint positions

### Synchronized Operations

1. Configure one arm as the Leader and others as Followers
2. Program or control the Leader arm to perform the desired sequence
3. All Follower arms will execute the same sequence simultaneously
4. This is particularly useful for production lines or educational demonstrations

## Python Example

The following Python script demonstrates how to configure and use the Leader-Follower Mode:

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
    time.sleep(5)  # Allow time for restart

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
    time.sleep(5)  # Allow time for restart

def get_mac_address(ip_address):
    """Get the MAC address of the arm."""
    cmd = {
        "type": "GetStatus"
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    status = json.loads(response.text)
    return status.get("mac", "")

def move_leader_arm(ip_address, joint_id, angle, speed):
    """Move a specific joint on the Leader arm."""
    cmd = {
        "type": "AngleCtrl",
        "id": joint_id,
        "angle": angle,
        "speed": speed
    }
    url = f"http://{ip_address}/js?json={json.dumps(cmd)}"
    response = requests.get(url)
    print(f"Movement command response: {response.text}")

# Example usage
if __name__ == "__main__":
    # Note: You would need to connect to each arm separately
    leader_ip = "192.168.4.1"
    follower_ip = "192.168.4.1"
    
    # Step 1: Get the Leader's MAC address
    leader_mac = get_mac_address(leader_ip)
    print(f"Leader MAC address: {leader_mac}")
    
    # Step 2: Configure the Leader
    configure_leader_mode(leader_ip)
    
    # Step 3: Configure the Follower
    configure_follower_mode(follower_ip, leader_mac)
    
    # Step 4: Move the Leader arm (Follower will automatically follow)
    move_leader_arm(leader_ip, 1, 1.57, 50)  # Move base joint to 90 degrees
    time.sleep(2)
    move_leader_arm(leader_ip, 2, 0.78, 30)  # Move shoulder joint to 45 degrees
    time.sleep(2)
    move_leader_arm(leader_ip, 3, 1.57, 40)  # Move elbow joint to 90 degrees
```

## Troubleshooting

### Common Issues

1. **Follower Not Connecting**
   - Verify the Leader's MAC address is correctly entered
   - Move the arms closer together
   - Restart both arms
   - Check for sources of interference

2. **Erratic Movement**
   - Reduce distance between arms
   - Move away from sources of interference
   - Slow down the Leader's movements
   - Check for mechanical issues in the Follower arm

3. **Connection Lost During Operation**
   - Move arms closer together
   - Check power supplies
   - Restart both arms
   - Reconfigure the connection

## Performance Considerations

- **Range**: Typical effective range of 30-50 meters in open areas
- **Latency**: Total system latency is typically 20-50ms
- **Power Consumption**: Higher than standard operation due to wireless communication
- **Interference**: Other 2.4GHz devices can affect performance

## Additional Resources

- [Detailed Leader-Follower Mode Documentation](../research/software/Leader_Follower_Mode.md)
- [ESP-NOW Protocol Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [RoArm-M3 Wiki Page](https://www.waveshare.com/wiki/RoArm-M3)
- [JSON Command System Documentation](../research/software/JSON_Command_System.md)
- [Python API Documentation](../research/software/Python_API.md)
