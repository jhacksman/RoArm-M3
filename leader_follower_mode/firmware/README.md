# RoArm-M3 Pro Firmware Installation Guide

This directory contains the firmware files necessary for the RoArm-M3 Pro robotic arm, with a specific focus on enabling and configuring the Leader-Follower mode functionality.

## Directory Structure

- `esp32/`: Contains the ESP32 firmware for the main control board
- `control_board/`: Contains Python scripts for communicating with the control board

## Prerequisites

Before installing the firmware, ensure you have the following:

### Hardware Requirements
- RoArm-M3 Pro robotic arm
- USB Type-C cable for connecting to the arm
- Computer with Windows, macOS, or Linux

### Software Requirements
- [Arduino IDE](https://www.arduino.cc/en/software) (version 1.8.19 or later)
- [Python 3.6+](https://www.python.org/downloads/) for running the control scripts
- [PlatformIO](https://platformio.org/install) (optional, alternative to Arduino IDE)

### Required Arduino Libraries
- ArduinoJson (version 6.x)
- SCServo
- ESP32 board support package

## ESP32 Firmware Installation

The ESP32 firmware is responsible for the core functionality of the RoArm-M3 Pro, including the Leader-Follower mode.

### Installing with Arduino IDE

1. **Install Required Libraries**:
   - Open Arduino IDE
   - Go to `Sketch > Include Library > Manage Libraries`
   - Search for and install:
     - ArduinoJson (version 6.x)
     - SCServo (may need to be downloaded separately)

2. **Install ESP32 Board Support**:
   - Go to `File > Preferences`
   - Add the following URL to the "Additional Boards Manager URLs" field:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to `Tools > Board > Boards Manager`
   - Search for "esp32" and install the ESP32 board package

3. **Open the Firmware Project**:
   - Navigate to the `esp32/RoArm-M3_example-250108/RoArm-M3_example` directory
   - Open the `.ino` file in Arduino IDE

4. **Configure Board Settings**:
   - Select `Tools > Board > ESP32 Arduino > ESP32 Dev Module`
   - Set the following options:
     - Flash Mode: "QIO"
     - Flash Size: "4MB (32Mb)"
     - Partition Scheme: "Default 4MB with spiffs"
     - CPU Frequency: "240MHz"
     - Upload Speed: "921600"

5. **Connect the RoArm-M3 Pro**:
   - Connect the RoArm-M3 Pro to your computer using a USB Type-C cable
   - Select the correct port under `Tools > Port`

6. **Upload the Firmware**:
   - Click the upload button (right arrow icon) or select `Sketch > Upload`
   - Wait for the upload to complete (you should see "Done uploading" in the status bar)

### Installing with PlatformIO (Alternative)

1. **Install PlatformIO**:
   - Follow the installation instructions at [platformio.org](https://platformio.org/install)

2. **Open the Project**:
   - Open PlatformIO IDE
   - Select "Open Project" and navigate to the `esp32/RoArm-M3_example-250108/RoArm-M3_example` directory

3. **Build and Upload**:
   - Connect the RoArm-M3 Pro to your computer
   - Click the "Upload" button in the PlatformIO toolbar
   - Wait for the upload to complete

## Control Board Python Scripts

The Python scripts in the `control_board` directory allow you to communicate with the RoArm-M3 Pro and configure the Leader-Follower mode.

### Setting Up Python Environment

1. **Install Required Python Packages**:
   ```bash
   cd leader_follower_mode/firmware/control_board/RoArm-M3_Python
   pip install -r requirements.txt
   ```

### Using the HTTP Control Script

The `http_simple_ctrl.py` script allows you to send JSON commands to the RoArm-M3 Pro over HTTP:

1. **Connect to the RoArm-M3 Pro WiFi**:
   - Power on the RoArm-M3 Pro
   - Connect to the WiFi network named "RoArm-M3" (password: 12345678)

2. **Run the HTTP Control Script**:
   ```bash
   python http_simple_ctrl.py 192.168.4.1
   ```

3. **Send Leader-Follower Mode Commands**:
   - To get the MAC address: `{"T":302}`
   - To set Leader mode: `{"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}`
   - To set Follower mode: `{"T":301,"mode":2,"dev":0,"cmd":0,"megs":0}`
   - To add a Follower to the Leader: `{"T":303,"mac":"XX:XX:XX:XX:XX:XX"}`
   - To enable broadcasting: `{"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}`

### Using the Serial Control Script

The `serial_simple_ctrl.py` script allows you to send JSON commands to the RoArm-M3 Pro over a serial connection:

1. **Connect the RoArm-M3 Pro**:
   - Connect the RoArm-M3 Pro to your computer using a USB Type-C cable
   - Identify the serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

2. **Run the Serial Control Script**:
   ```bash
   python serial_simple_ctrl.py /dev/ttyUSB0  # Replace with your port
   ```

3. **Send Leader-Follower Mode Commands**:
   - Same JSON commands as with the HTTP control script

## Configuring Leader-Follower Mode

After installing the firmware, follow these steps to configure the Leader-Follower mode:

### Setting Up a Leader Arm

1. Get the MAC address of the Leader arm:
   ```
   {"T":302}
   ```

2. Configure the arm as a Leader:
   ```
   {"T":301,"mode":1,"dev":0,"cmd":0,"megs":0}
   ```

3. Enable broadcasting to Followers:
   ```
   {"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}
   ```

### Setting Up a Follower Arm

1. Get the MAC address of the Follower arm:
   ```
   {"T":302}
   ```

2. On the Leader arm, add the Follower's MAC address:
   ```
   {"T":303,"mac":"XX:XX:XX:XX:XX:XX"}  # Replace with Follower's MAC
   ```

3. On the Follower arm, add the Leader's MAC address:
   ```
   {"T":303,"mac":"YY:YY:YY:YY:YY:YY"}  # Replace with Leader's MAC
   ```

4. Configure the arm as a Follower:
   ```
   {"T":301,"mode":2,"dev":0,"cmd":0,"megs":0}
   ```

## Troubleshooting

### Common Issues

1. **Upload Fails**:
   - Ensure the RoArm-M3 Pro is properly connected
   - Try pressing the reset button on the ESP32 board just before uploading
   - Check that you've selected the correct port and board settings

2. **WiFi Connection Issues**:
   - Ensure the RoArm-M3 Pro is powered on
   - Check that you're connecting to the correct WiFi network
   - Try restarting the RoArm-M3 Pro

3. **Leader-Follower Mode Not Working**:
   - Verify that both arms have the correct firmware
   - Ensure the MAC addresses are correctly configured
   - Check that both arms are powered on and within range
   - Verify that the Leader has broadcasting enabled

### Getting Help

If you encounter issues not covered in this guide, refer to the following resources:

- [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3)
- [GitHub Repository Issues](https://github.com/jhacksman/RoArm-M3/issues)

## Additional Resources

- [Leader-Follower Mode Documentation](../README.md): Detailed information about the Leader-Follower mode
- [JSON Command System Documentation](../../research/software/JSON_Command_System.md): Complete reference for all JSON commands
