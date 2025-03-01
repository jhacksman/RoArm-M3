# RoArm-M3 Pro Configuration Persistence Guide

This document explains how to save configuration settings persistently on the RoArm-M3 Pro robotic arm so they remain after power cycling.

## Overview

The RoArm-M3 Pro uses two main mechanisms for configuration persistence:

1. **LittleFS** - A file system for embedded devices that stores files in flash memory
2. **NVS (Non-Volatile Storage)** - ESP32's built-in key-value storage system

When you change settings through the web interface or via JSON commands, these changes are applied to the current session but are not automatically saved to persistent storage. You must explicitly save configurations using specific JSON commands.

## WiFi Configuration Persistence

To save WiFi settings persistently, use one of the following JSON commands:

### Save Current WiFi Settings

```json
{"T":406}
```

This command (`CMD_WIFI_CONFIG_CREATE_BY_STATUS`) creates a `wifiConfig.json` file in flash memory using the current WiFi settings. Use this after changing settings with other WiFi commands.

### Save Custom WiFi Settings

```json
{"T":407,"mode":3,"ap_ssid":"RoArm-M3","ap_password":"12345678","sta_ssid":"YourWifi","sta_password":"YourPassword"}
```

This command (`CMD_WIFI_CONFIG_CREATE_BY_INPUT`) creates a `wifiConfig.json` file with custom settings:
- `mode`: WiFi mode (0=OFF, 1=AP, 2=STA, 3=AP+STA)
- `ap_ssid`: Access Point SSID
- `ap_password`: Access Point password
- `sta_ssid`: Station mode SSID (your WiFi network)
- `sta_password`: Station mode password

## WiFi Mode Configuration

To set the WiFi mode that will be used on boot:

```json
{"T":401,"cmd":3}
```

Where `cmd` can be:
- `0`: OFF
- `1`: AP (Access Point)
- `2`: STA (Station)
- `3`: AP+STA (both modes)

**Important**: After setting the WiFi mode, you must save the configuration using the `{"T":406}` command.

## ESP-NOW Configuration Persistence

ESP-NOW settings (for Leader-Follower mode) are also stored in flash memory. After configuring ESP-NOW settings with commands like:

```json
{"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}  // Set Leader mode
```

or 

```json
{"T":301,"mode":2,"dev":0,"cmd":0,"megs":0}  // Set Follower mode
```

You must save the configuration using:

```json
{"T":406}
```

This ensures that the ESP-NOW settings persist after power cycling.

## How Configuration Persistence Works

1. **Initial Configuration**: When you send JSON commands to change settings, they are applied to the current session.
2. **Saving Configuration**: When you send the `{"T":406}` command, the current settings are saved to a file in flash memory.
3. **Loading Configuration**: When the RoArm-M3 Pro boots up, it checks for configuration files in flash memory and loads them.

The `wifiConfig.json` file contains the following information:
- WiFi mode on boot
- AP SSID and password
- STA SSID and password

## Hardware Requirements

No additional hardware (such as a microSD card) is required for configuration persistence. The RoArm-M3 Pro uses the ESP32's internal flash memory for storage.

## Troubleshooting

If configurations are still not persisting after power cycling:

1. Ensure you're sending the save command (`{"T":406}`) after changing settings
2. Try clearing the NVS (Non-Volatile Storage) and reconfiguring:
   ```json
   {"T":604}
   ```
3. Reboot the device after saving configurations:
   ```json
   {"T":600}
   ```
4. Check the free flash space to ensure there's enough room for configuration files:
   ```json
   {"T":601}
   ```
5. If you're still having issues, try setting up the configuration from scratch:
   - Clear NVS: `{"T":604}`
   - Set WiFi mode: `{"T":401,"cmd":3}`
   - Configure AP: `{"T":402,"ssid":"RoArm-M3","password":"12345678"}`
   - Configure STA: `{"T":403,"ssid":"YourWifi","password":"YourPassword"}`
   - Save configuration: `{"T":406}`
   - Reboot: `{"T":600}`

## Example Workflow

Here's a complete example workflow for configuring WiFi settings persistently:

1. Connect to the RoArm-M3 Pro via the web interface or serial connection
2. Set the WiFi mode to AP+STA:
   ```json
   {"T":401,"cmd":3}
   ```
3. Configure your custom AP settings:
   ```json
   {"T":402,"ssid":"MyRoArm","password":"securepassword"}
   ```
4. Configure your STA settings to connect to your home WiFi:
   ```json
   {"T":403,"ssid":"HomeWiFi","password":"wifipassword"}
   ```
5. Save the configuration:
   ```json
   {"T":406}
   ```
6. Reboot the device to verify that settings persist:
   ```json
   {"T":600}
   ```

After the device reboots, it should connect to your home WiFi network and also create an access point with your custom settings.
