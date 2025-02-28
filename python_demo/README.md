# RoArm-M3 Python Demo Files

This directory contains Python demo files for controlling the RoArm-M3 Pro robotic arm. These files demonstrate how to communicate with the arm using both HTTP and serial interfaces, allowing you to send JSON commands programmatically.

## Directory Structure

```
python_demo/
├── original/                  # Original files from Waveshare
│   ├── http_simple_ctrl.py    # Original HTTP control example
│   ├── requirements.txt       # Original dependencies list
│   ├── serial_simple_ctrl.py  # Original serial control example
│   └── RoArm-M3_Python.zip    # Original zip file for reference
│
├── enhanced/                  # Enhanced versions with additional documentation
│   ├── http/                  # HTTP communication examples
│   │   └── http_simple_ctrl.py
│   ├── serial/                # Serial communication examples
│   │   └── serial_simple_ctrl.py
│   ├── examples/              # Additional example scripts
│   └── requirements.txt       # Dependencies list
│
└── README.md                  # This file
```

## Original Files from Waveshare

The following files were provided by Waveshare in the [RoArm-M3_Python.zip](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip) package:

### requirements.txt

This file lists the Python dependencies required to run the demo scripts:

```
certifi==2023.7.22
charset-normalizer==3.2.0
customtkinter==5.2.0
darkdetect==0.8.0
idna==3.4
pyserial==3.5
requests==2.31.0
urllib3==2.0.4
```

Key dependencies:
- `requests`: Used for HTTP communication
- `pyserial`: Used for serial communication
- `customtkinter`: Used for GUI elements (though not used in the basic demos)

### http_simple_ctrl.py

A simple Python script that demonstrates how to send JSON commands to the RoArm-M3 via HTTP. This script:

1. Takes an IP address as a command-line argument
2. Prompts the user to input JSON commands
3. Sends the commands to the arm via HTTP GET requests
4. Displays the response from the arm

Usage:
```bash
python http_simple_ctrl.py 192.168.4.1
```

Example JSON commands to try:
```
{"type":"AngleCtrl","id":1,"angle":90,"speed":50}
{"type":"GetStatus"}
```

### serial_simple_ctrl.py

A simple Python script that demonstrates how to send JSON commands to the RoArm-M3 via serial connection. This script:

1. Takes a serial port name as a command-line argument
2. Creates a separate thread to continuously read and display responses from the arm
3. Prompts the user to input JSON commands
4. Sends the commands to the arm via the serial connection

Usage:
```bash
python serial_simple_ctrl.py /dev/ttyUSB0  # Linux
python serial_simple_ctrl.py COM3          # Windows
```

Example JSON commands to try:
```
{"type":"AngleCtrl","id":1,"angle":90,"speed":50}
{"type":"GetStatus"}
```

## Enhanced Versions

The enhanced versions in the `enhanced/` directory build upon the original examples with:

1. Additional documentation and comments
2. Error handling improvements
3. More comprehensive examples
4. Better user interface feedback

These enhanced versions maintain compatibility with the original functionality while providing a more robust and user-friendly experience.

## Installation and Setup

To use these demo files:

1. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Connect to the RoArm-M3:
   - For HTTP: Connect to the arm's WiFi network (SSID: RoArm-M3, Password: 12345678)
   - For Serial: Connect the arm to your computer using a USB cable

3. Run the appropriate script:
   - For HTTP: `python http_simple_ctrl.py 192.168.4.1`
   - For Serial: `python serial_simple_ctrl.py /dev/ttyUSB0` (replace with your port)

## JSON Command System

Both demo scripts allow you to send JSON commands to the arm. The RoArm-M3 uses a JSON-based command system for control. Here are some example commands:

### Basic Movement
```json
{"type":"AngleCtrl","id":1,"angle":90,"speed":50}
```
This moves servo #1 to the 90-degree position at 50% speed.

### Get Status
```json
{"type":"GetStatus"}
```
This returns the current status of the arm, including joint positions.

### Coordinate Control
```json
{"type":"CoordCtrl","x":150,"y":0,"z":100,"speed":50}
```
This moves the end-effector to the specified coordinates.

For a complete list of available commands, refer to the [JSON Command System documentation](../research/software/JSON_Command_System.md).

## Additional Resources

- [RoArm-M3 Wiki Page](https://www.waveshare.com/wiki/RoArm-M3)
- [JSON Command System Documentation](../research/software/JSON_Command_System.md)
- [Python API Documentation](../research/software/Python_API.md)

## Contributing

To contribute enhanced examples or improvements:

1. Create your new example in the appropriate subdirectory of `enhanced/`
2. Add documentation explaining what your example does
3. Ensure your code follows the existing style and includes proper error handling
4. Update this README.md if necessary to reflect your additions

## License

These demo files are provided by Waveshare and are subject to their licensing terms. The enhanced versions and additional documentation are provided under the same terms as the original repository.
