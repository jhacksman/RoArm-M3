# Control Board Scripts for RoArm-M3 Pro

This directory contains Python scripts for communicating with the RoArm-M3 Pro control board, with a focus on configuring and using the Leader-Follower mode.

## Scripts Overview

The following scripts are included:

- `http_simple_ctrl.py`: Sends JSON commands to the RoArm-M3 Pro over HTTP
- `serial_simple_ctrl.py`: Sends JSON commands to the RoArm-M3 Pro over a serial connection
- `requirements.txt`: Lists the required Python packages

## Prerequisites

Before using these scripts, ensure you have:

- Python 3.6 or later installed
- Required Python packages installed (`pip install -r requirements.txt`)
- RoArm-M3 Pro with the latest firmware

## HTTP Control

The `http_simple_ctrl.py` script allows you to send JSON commands to the RoArm-M3 Pro over HTTP:

### Usage

1. Connect to the RoArm-M3 Pro WiFi (SSID: RoArm-M3, Password: 12345678)
2. Run the script:
   ```bash
   python http_simple_ctrl.py 192.168.4.1
   ```
3. Enter JSON commands when prompted

### Example Commands for Leader-Follower Mode

- Get MAC address:
  ```json
  {"T":302}
  ```

- Set Leader mode:
  ```json
  {"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}
  ```

- Enable broadcasting:
  ```json
  {"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}
  ```

- Add a Follower to the Leader:
  ```json
  {"T":303,"mac":"XX:XX:XX:XX:XX:XX"}
  ```

- Set Follower mode:
  ```json
  {"T":301,"mode":2,"dev":0,"cmd":0,"megs":0}
  ```

## Serial Control

The `serial_simple_ctrl.py` script allows you to send JSON commands to the RoArm-M3 Pro over a serial connection:

### Usage

1. Connect the RoArm-M3 Pro to your computer using a USB Type-C cable
2. Run the script with the appropriate serial port:
   ```bash
   python serial_simple_ctrl.py /dev/ttyUSB0  # Replace with your port
   ```
3. Enter JSON commands when prompted

### Example Commands

The same JSON commands used with the HTTP control script can be used with the serial control script.

## Extending the Scripts

If you need to extend the functionality of these scripts:

1. Create a new Python file based on one of the existing scripts
2. Add your custom functionality
3. Ensure you maintain the JSON command format for compatibility with the RoArm-M3 Pro

## Troubleshooting

### Common Issues

1. **Connection Errors**:
   - Ensure the RoArm-M3 Pro is powered on
   - Check that you're connected to the correct WiFi network (for HTTP)
   - Verify the serial port is correct (for serial)

2. **Command Not Recognized**:
   - Ensure the JSON format is correct
   - Check that the command is supported by the firmware version

3. **No Response**:
   - Verify the RoArm-M3 Pro is powered on
   - Check the connection (WiFi or serial)
   - Try restarting the RoArm-M3 Pro

### Getting Help

If you encounter issues not covered here, refer to the [main firmware README](../README.md) for additional troubleshooting steps and resources.
