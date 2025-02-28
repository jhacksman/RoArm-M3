# RoArm-M3 Pro Component List

This document lists all the major components of the RoArm-M3 Pro robotic arm that require detailed research and documentation.

## Main Components

1. **ESP32-WROOM-32 Main Control Module**
   - Microcontroller for the robotic arm
   - Can be developed using Arduino IDE
   - Needs detailed documentation on programming and capabilities

2. **ST3235 Metal Shell Servos (Pro Version)**
   - Used in all joints except the gripper in the Pro version
   - Requires documentation on specifications, control methods, and limitations

3. **ST3215 Plastic Shell Servo**
   - Used in the gripper joint
   - Requires documentation on specifications and control methods

4. **General Driver Board for Robots**
   - Main control board with various interfaces
   - Needs detailed documentation on all onboard components and connections

5. **INA219 Voltage/Current Monitoring Chip**
   - Used for power monitoring
   - Requires documentation on usage and integration

6. **TB6612FNG Motor Control Chip**
   - Used for motor control
   - Needs documentation on specifications and usage

7. **AK09918C 3-axis Electronic Compass**
   - Used for orientation sensing
   - Requires documentation on integration and usage

8. **QMI8658 6-axis Motion Sensor**
   - Used for motion detection
   - Needs documentation on capabilities and integration

9. **CP2102 Serial-to-USB Chips**
   - Used for communication
   - Requires documentation on drivers and usage

## Software Components

1. **Web Interface**
   - Browser-based control system
   - Needs documentation on all features and usage

2. **JSON Command System**
   - Protocol for controlling the arm
   - Requires comprehensive documentation on all available commands

3. **Python API**
   - Libraries for controlling the arm via Python
   - Needs documentation on installation and usage

4. **ROS2 Integration**
   - Robot Operating System compatibility
   - Requires documentation on setup and usage

5. **LeRobot AI Project Integration**
   - AI capabilities for the robotic arm
   - Needs detailed documentation on setup and usage

## Control Features

1. **AngleCtrl: Servo Angle Control**
   - Direct control of individual servo angles
   - Requires documentation on usage and limitations

2. **Torque Lock Control**
   - Control of servo torque locking
   - Needs documentation on functionality and use cases

3. **DEFA: Dynamic External Force Adaptive Control**
   - Adaptive control system
   - Requires documentation on functionality and applications

4. **COORDCTRL: End Point Coordinate Control**
   - Inverse kinematics control system
   - Needs detailed documentation on usage and coordinate system

5. **Leader-Follower Mode**
   - Priority research area
   - Requires comprehensive documentation on setup and usage

## Research Priority

The following components should be prioritized for research:

1. **Leader-Follower Mode** - Highest priority as specifically requested
2. **ST3235 Metal Shell Servos** - Key differentiator of the Pro version
3. **ESP32 Control Module** - Core of the system's functionality
4. **JSON Command System** - Essential for programmatic control
5. **General Driver Board** - Central to understanding the hardware architecture

Each component will have its own dedicated markdown file in the research folder with comprehensive information.
