"""
Configuration file for RoArm-M3 Isaac Sim Bridge

This file contains all the configuration parameters for the RoArm-M3 Isaac Sim Bridge.
"""

# Serial communication settings
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.01  # 10ms timeout for 100Hz data

# Device identification settings
USE_MAC_ADDRESS = True  # Use MAC address for device identification
USE_SSID = False        # Use SSID for device identification

# Camera settings
CAMERA_INDEX = 0        # Default camera index
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Isaac Sim settings
ISAAC_HOST = "localhost"
ISAAC_PORT = 8000

# Data processing settings
BUFFER_SIZE = 100       # Number of data points to buffer
SMOOTHING_FACTOR = 0.1  # Exponential smoothing factor

# Logging settings
LOG_LEVEL = "INFO"
LOG_FILE = "roarm_bridge.log"

# RoArm-M3 specific settings
# These are the field names in the JSON telemetry data
TELEMETRY_FIELDS = [
    "T",    # Transaction/telemetry type identifier
    "x",    # X coordinate (mm)
    "y",    # Y coordinate (mm)
    "z",    # Z coordinate (mm)
    "tit",  # Tilt angle (radians)
    "b",    # Base joint angle (radians)
    "s",    # Shoulder joint angle (radians)
    "e",    # Elbow joint angle (radians)
    "t",    # Wrist joint angle (radians)
    "r",    # Roll joint angle (radians)
    "g",    # Gripper joint angle (radians)
    "tB",   # Base joint load/temperature
    "tS",   # Shoulder joint load/temperature
    "tE",   # Elbow joint load/temperature
    "tT",   # Wrist joint load/temperature
    "tR",   # Roll joint load/temperature
    "tG"    # Gripper joint load/temperature
]

# Auto-provisioning settings
AUTO_PROVISION = True   # Automatically provision new devices
DEVICE_TIMEOUT = 5.0    # Time to wait for device to respond (seconds)

# Device reset and MAC address detection settings
RESET_TIMEOUT = 10.0    # Time to wait for device to reset (seconds)
MAC_DETECTION_TIMEOUT = 5.0  # Time to wait for MAC address detection (seconds)
RETRY_ATTEMPTS = 3      # Number of retry attempts for device operations
RETRY_DELAY = 1.0       # Delay between retry attempts (seconds)

# Command definitions
CMD_REBOOT = 600        # Command to reboot the device
CMD_GET_MAC_ADDRESS = 302  # Command to get the MAC address
