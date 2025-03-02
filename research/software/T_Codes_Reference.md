# RoArm-M3 Pro T Codes Reference

## Overview

The RoArm-M3 Pro robotic arm uses a JSON command system where commands are identified by a "T" value. These "T codes" represent different functions and capabilities of the arm. This document provides a comprehensive reference for all available T codes, organized by category and numerical order.

## Etymology of "T" Codes

The "T" in T codes stands for "Type" or "Task," indicating the type of command or task to be performed. This naming convention is used throughout the RoArm-M3 Pro firmware to categorize different command functions.

## T Codes by Category

### Movement Control (100-199)

| T Code | Name | Description | Example | Parameters |
|--------|------|-------------|---------|------------|
| 101 | CMD_SINGLE_JOINT_CTRL | Controls a single joint/servo | `{"T":101,"joint":0,"rad":1.57,"spd":50,"acc":10}` | joint: Joint ID (0-5)<br>rad: Angle in radians<br>spd: Speed (1-100)<br>acc: Acceleration (1-100) |
| 102 | CMD_MULTI_JOINT_CTRL | Controls multiple joints simultaneously | `{"T":102,"joints":[0,1,2],"rads":[1.57,0.78,1.57],"spd":50,"acc":10}` | joints: Array of joint IDs<br>rads: Array of angles in radians<br>spd: Speed (1-100)<br>acc: Acceleration (1-100) |
| 103 | CMD_ALL_JOINT_CTRL | Controls all joints at once | `{"T":103,"rads":[1.57,0.78,1.57,0,0,1.57],"spd":50,"acc":10}` | rads: Array of 6 angles in radians<br>spd: Speed (1-100)<br>acc: Acceleration (1-100) |
| 110 | CMD_COORDCTRL | Controls end-effector position in Cartesian space | `{"T":110,"x":150,"y":0,"z":100,"roll":0,"pitch":90,"yaw":0,"spd":50}` | x,y,z: Coordinates in mm<br>roll,pitch,yaw: Orientation in degrees<br>spd: Speed (1-100) |

### Mission Control (200-299)

| T Code | Name | Description | Example | Parameters |
|--------|------|-------------|---------|------------|
| 210 | CMD_TORQUE_CTRL | Controls servo torque | `{"T":210,"cmd":0}` | cmd: 0=Disable, 1=Enable |
| 220 | CMD_CREATE_MISSION | Creates a new mission | `{"T":220,"name":"mission1","intro":"My first mission"}` | name: Mission name<br>intro: Mission description |
| 222 | CMD_APPEND_MISSION_STEP | Adds a step to a mission | `{"T":222,"name":"mission1","step":"{\"T\":101,\"joint\":0,\"rad\":1.57,\"spd\":50,\"acc\":10}"}` | name: Mission name<br>step: JSON string of the command to add (must escape quotes) |
| 230 | CMD_RUN_MISSION | Runs a saved mission | `{"T":230,"name":"mission1"}` | name: Mission name |
| 240 | CMD_LIST_MISSIONS | Lists all saved missions | `{"T":240}` | None |
| 242 | CMD_DELETE_MISSION | Deletes a mission | `{"T":242,"name":"mission1"}` | name: Mission name |

### ESP-NOW Communication (300-399)

| T Code | Name | Description | Example | Parameters |
|--------|------|-------------|---------|------------|
| 300 | CMD_BROADCAST_FOLLOWER | Enables/disables broadcasting to followers | `{"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}` | mode: 0=Disable, 1=Enable<br>mac: MAC address (FF:FF:FF:FF:FF:FF for broadcast) |
| 301 | CMD_ESP_NOW_CONFIG | Configures ESP-NOW mode | `{"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}` | mode: 0=Disabled, 1=F-LEADER-B (Broadcast), 2=F-LEADER-S (Single), 3=FOLLOWER<br>dev: Device ID<br>cmd: Command<br>megs: Message |
| 302 | CMD_GET_MAC_ADDRESS | Retrieves the MAC address of the arm | `{"T":302}` | None |
| 303 | CMD_ESP_NOW_ADD_FOLLOWER | Adds a Follower to the Leader's list | `{"T":303,"mac":"XX:XX:XX:XX:XX:XX"}` | mac: MAC address of the Follower/Leader to add |
| 304 | CMD_ESP_NOW_REMOVE_FOLLOWER | Removes a Follower from the Leader's list | `{"T":304,"mac":"XX:XX:XX:XX:XX:XX"}` | mac: MAC address of the Follower to remove |
| 305 | CMD_ESP_NOW_MANY_CTRL | Controls all connected Followers | `{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":""}` | dev: Device ID<br>b,s,e,t,r,h: Joint positions for base, shoulder, elbow, wrist, roll, hand<br>cmd: Command<br>megs: Message |
| 306 | CMD_ESP_NOW_SINGLE | Controls a specific Follower | `{"T":306,"mac":"XX:XX:XX:XX:XX:XX","dev":0,"b":0,"s":0,"e":1.57,"t":0,"r":0,"h":1.57,"cmd":0,"megs":""}` | mac: MAC address of the specific Follower<br>dev: Device ID<br>b,s,e,t,r,h: Joint positions<br>cmd: Command<br>megs: Message |

### WiFi Configuration (400-499)

| T Code | Name | Description | Example | Parameters |
|--------|------|-------------|---------|------------|
| 401 | CMD_WIFI_ON_BOOT | Sets the WiFi mode on boot | `{"T":401,"cmd":3}` | cmd: 0=OFF, 1=AP, 2=STA, 3=AP+STA |
| 406 | CMD_WIFI_CONFIG_CREATE_BY_STATUS | Creates a wifiConfig.json file from current settings | `{"T":406}` | None |
| 407 | CMD_WIFI_CONFIG_CREATE_BY_INPUT | Creates a wifiConfig.json file from provided parameters | `{"T":407,"mode":3,"ap_ssid":"RoArm-M3","ap_password":"12345678","sta_ssid":"YourWifi","sta_password":"YourPassword"}` | mode: WiFi mode (0-3)<br>ap_ssid: Access Point SSID<br>ap_password: Access Point password<br>sta_ssid: Station SSID<br>sta_password: Station password |

### System Commands (600-699)

| T Code | Name | Description | Example | Parameters |
|--------|------|-------------|---------|------------|
| 600 | CMD_REBOOT | Reboots the device | `{"T":600}` | None |
| 602 | CMD_BOOT_MISSION_INFO | Views the current boot mission content | `{"T":602}` | None |
| 603 | CMD_RESET_BOOT_MISSION | Resets the boot mission | `{"T":603}` | None |
| 604 | CMD_NVS_CLEAR | Clears the NVS (Non-Volatile Storage) | `{"T":604}` | None |

## Usage Examples

### Movement Control Example

To move the base joint (joint 0) to 90 degrees (1.57 radians) at 50% speed:

```json
{"T":101,"joint":0,"rad":1.57,"spd":50,"acc":10}
```

### Leader-Follower Mode Configuration Example

To configure an arm as a Leader in F-LEADER-B (Broadcast) mode:

```json
{"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}
```

To enable broadcasting to Followers:

```json
{"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}
```

To add a Follower to the Leader's list:

```json
{"T":303,"mac":"XX:XX:XX:XX:XX:XX"}
```

### WiFi Configuration Example

To save the current WiFi configuration:

```json
{"T":406}
```

To set the WiFi mode on boot to AP+STA:

```json
{"T":401,"cmd":3}
```

### Boot Mission Example

To reset the boot mission:

```json
{"T":603}
```

To create a new boot mission:

```json
{"T":220,"name":"boot","intro":"boot mission"}
```

To add a step to the boot mission:

```json
{"T":222,"name":"boot","step":"{\"T\":301,\"mode\":1,\"dev\":0,\"cmd\":0,\"megs\":0}"}
```

## Best Practices

1. **Command Validation**: Always verify that the T code and parameters are correct before sending
2. **Error Handling**: Implement proper error handling for command responses
3. **Sequential Commands**: For complex operations, send commands in the correct sequence
4. **Parameter Ranges**: Ensure parameters are within valid ranges
5. **Persistence**: Use configuration persistence commands (T:406, T:407) to save settings

## Sources

This documentation is based on information from the following sources:

1. **Waveshare Wiki**: The [RoArm-M3 Wiki Page](https://www.waveshare.com/wiki/RoArm-M3) provides basic information about the JSON command system
2. **Firmware Analysis**: Examination of the RoArm-M3 Pro firmware reveals the complete set of T codes and their functions
3. **Leader-Follower Mode Documentation**: The [Leader-Follower Mode Documentation](./Leader_Follower_Mode.md) provides details on ESP-NOW related T codes
4. **JSON Command System Documentation**: The [JSON Command System Documentation](./JSON_Command_System.md) provides an overview of the command structure
5. **Boot Mission System Documentation**: The [Boot Mission System Documentation](./boot_mission/Boot_Mission_System.md) explains the boot mission-related T codes

## Conclusion

The T codes system provides a comprehensive and flexible interface for controlling all aspects of the RoArm-M3 Pro robotic arm. By understanding the different categories and functions of T codes, users can fully leverage the capabilities of the arm for a wide range of applications.
