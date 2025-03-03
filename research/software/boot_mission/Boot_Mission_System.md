# RoArm-M3 Pro Boot Mission System

This document explains the Boot Mission System of the RoArm-M3 Pro robotic arm, which can be used as a workaround for saving ESP-NOW settings (Leader-Follower mode) persistently.

## Overview

The Boot Mission System allows you to create a sequence of commands that will be automatically executed when the RoArm-M3 Pro boots up. This feature can be used to reconfigure ESP-NOW settings (for Leader-Follower mode) after each power cycle, effectively providing a workaround for the lack of direct ESP-NOW configuration persistence.

## How the Boot Mission System Works

1. The RoArm-M3 Pro stores a special mission file called `boot.mission` in its flash memory
2. When the device boots up, it automatically executes the commands stored in this file
3. You can add any valid JSON commands to this mission, including ESP-NOW configuration commands

## Boot Mission Commands

The following JSON commands are used to manage the boot mission:

| Command | T Value | Description | Example |
|---------|---------|-------------|---------|
| CMD_CREATE_MISSION | 220 | Creates a new mission | `{"T":220,"name":"boot","intro":"boot mission"}` |
| CMD_APPEND_MISSION_STEP | 222 | Adds a step to a mission | `{"T":222,"name":"boot","step":"{\"T\":101,\"joint\":0,\"rad\":1.57,\"spd\":50,\"acc\":10}"}` |
| CMD_BOOT_MISSION_INFO | 602 | View the current boot mission content | `{"T":602}` |
| CMD_RESET_BOOT_MISSION | 603 | Reset the boot mission (creates an empty boot mission) | `{"T":603}` |

## Creating a Boot Mission for ESP-NOW Configuration

To create a boot mission that configures ESP-NOW settings (Leader-Follower mode) at startup:

### Step 1: Reset the Boot Mission

First, reset the boot mission to start with a clean slate:

```json
{"T":603}
```

This command deletes the existing `boot.mission` file and creates a new empty one.

### Step 2: Create a New Boot Mission

After resetting the boot mission, you need to create a new one:

```json
{"T":220,"name":"boot","intro":"boot mission"}
```

This command creates a new mission named "boot" that will be executed at startup.

### Step 3: Add ESP-NOW Configuration Commands

Use the mission editing commands to add ESP-NOW configuration commands to the boot mission. For example:

#### For a Leader Arm

```json
// Append step to set Leader mode
{"T":222,"name":"boot","step":"{\"T\":301,\"mode\":1,\"dev\":0,\"cmd\":0,\"megs\":0}"}

// Append step to enable broadcasting
{"T":222,"name":"boot","step":"{\"T\":300,\"mode\":1,\"mac\":\"FF:FF:FF:FF:FF:FF\"}"}

// Append step to add a Follower (replace with actual MAC address)
{"T":222,"name":"boot","step":"{\"T\":303,\"mac\":\"XX:XX:XX:XX:XX:XX\"}"}
```

#### For a Follower Arm

```json
// Append step to set Follower mode
{"T":222,"name":"boot","step":"{\"T\":301,\"mode\":2,\"dev\":0,\"cmd\":0,\"megs\":0}"}

// Append step to add the Leader (replace with actual MAC address)
{"T":222,"name":"boot","step":"{\"T\":303,\"mac\":\"YY:YY:YY:YY:YY:YY\"}"}
```

### Step 3: Verify the Boot Mission

To verify that your boot mission has been correctly set up, use:

```json
{"T":602}
```

This will display the current content of the boot mission.

## Example Boot Mission for Leader-Follower Configuration

Here's a complete example of setting up a boot mission for a Leader arm:

1. Reset the boot mission:
   ```json
   {"T":603}
   ```

2. Create a new boot mission:
   ```json
   {"T":220,"name":"boot","intro":"boot mission"}
   ```

3. Add Leader mode configuration:
   ```json
   {"T":222,"name":"boot","step":"{\"T\":301,\"mode\":1,\"dev\":0,\"cmd\":0,\"megs\":0}"}
   ```

4. Enable broadcasting:
   ```json
   {"T":222,"name":"boot","step":"{\"T\":300,\"mode\":1,\"mac\":\"FF:FF:FF:FF:FF:FF\"}"}
   ```

5. Add a Follower (replace with actual MAC address):
   ```json
   {"T":222,"name":"boot","step":"{\"T\":303,\"mac\":\"CC:DB:A7:5B:E4:1C\"}"}
   ```

6. Verify the boot mission:
   ```json
   {"T":602}
   ```

## Important Notes

1. **Order Matters**: The commands in the boot mission are executed in the order they are added. Make sure to add them in the correct sequence.

2. **Creating the Mission First**: You must create the boot mission with the T:220 command before adding steps with T:222 commands. If you try to add steps without creating the mission first, the steps will not be saved.

3. **Escaping JSON**: When adding JSON commands to the boot mission, you need to escape the quotes with backslashes as shown in the examples.

4. **MAC Addresses**: You'll need to know the MAC addresses of your arms. Use the `{"T":302}` command to get the MAC address of an arm.

5. **Testing**: After setting up the boot mission, reboot the arm using `{"T":600}` to verify that the ESP-NOW settings are correctly configured at startup.

6. **Limitations**: The boot mission has a limited size. If you encounter issues, try keeping your boot mission as concise as possible.

## Troubleshooting

If your boot mission is not working as expected:

1. Verify the boot mission content using `{"T":602}`
2. Check for syntax errors in your JSON commands
3. Ensure the MAC addresses are correct
4. Make sure you've created the boot mission with T:220 before adding steps with T:222
5. Try resetting the boot mission with `{"T":603}` and recreating it
6. Reboot the device with `{"T":600}` after making changes

## Conclusion

While the RoArm-M3 Pro does not have a direct mechanism for saving ESP-NOW settings (Leader-Follower mode) persistently, the Boot Mission System provides an effective workaround. By adding ESP-NOW configuration commands to the boot mission, you can ensure that your Leader-Follower mode settings are automatically restored each time the arm powers up.
