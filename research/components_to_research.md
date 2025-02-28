# RoArm-M3 Pro Components to Research

This document lists all components of the RoArm-M3 Pro that need detailed research and documentation. Each component will have its own markdown file in the research folder.

## Hardware Components

### Main Control Components
1. **ESP32-WROOM-32 Main Control Module**
   - Primary microcontroller for the robotic arm
   - Can be developed using Arduino IDE
   - Research needed: specifications, programming interfaces, capabilities

2. **General Driver Board for Robots**
   - Main control board with various interfaces
   - Research needed: schematic analysis, pin mappings, capabilities

### Servo Motors
3. **ST3235 Metal Shell Servo (Pro Version)**
   - Used in all joints except the gripper in the Pro version
   - Research needed: specifications, torque ratings, control protocol, datasheets

4. **ST3215 Plastic Shell Servo (Gripper)**
   - Used in the gripper joint
   - Research needed: specifications, torque ratings, control protocol, datasheets

### Sensors and Chips
5. **INA219 Voltage/Current Monitoring Chip**
   - Used for power monitoring
   - Research needed: specifications, integration details, usage examples

6. **TB6612FNG Motor Control Chip**
   - Used for motor control
   - Research needed: specifications, control methods, limitations

7. **AK09918C 3-axis Electronic Compass**
   - Used for orientation sensing
   - Research needed: specifications, integration details, usage examples

8. **QMI8658 6-axis Motion Sensor**
   - Used for motion detection
   - Research needed: specifications, integration details, usage examples

9. **CP2102 Serial-to-USB Chip (Radar Data)**
   - Used for radar data transmission
   - Research needed: specifications, driver details, usage examples

10. **CP2102 Serial-to-USB Chip (ESP32 Communication)**
    - Used for ESP32 serial communication
    - Research needed: specifications, driver details, usage examples

### Communication Interfaces
11. **IPEX Gen 1 WIFI Interface**
    - Used for connecting antenna
    - Research needed: specifications, compatibility, usage examples

12. **Lidar Interface**
    - Integrated radar adapter board functionality
    - Research needed: specifications, compatibility, usage examples

13. **IIC Peripheral Expansion Interface**
    - Can connect OLED screens or other IIC sensors
    - Research needed: specifications, compatible devices, usage examples

14. **Type-C Port (LIDAR)**
    - LIDAR data interface
    - Research needed: specifications, compatible devices, usage examples

15. **Type-C Port (USB)**
    - ESP32 serial communication interface
    - Research needed: specifications, programming interface, usage examples

16. **40PIN Expansion Interface (Raspberry Pi/Jetson)**
    - Easy access to Raspberry Pi 4B, Pi Zero, or Jetson Orin Nano
    - Research needed: pin mappings, compatibility, usage examples

17. **40PIN Expansion Interface (Host Computer)**
    - For using pins on host computer installed on driver board
    - Research needed: pin mappings, compatibility, usage examples

18. **TF Card Slot**
    - Can store logs or WIFI configurations
    - Research needed: specifications, usage examples

### Power Components
19. **XH2.54 Power Connector**
    - Inputs DC7~12.6V, powers servos and motors
    - Research needed: specifications, compatible power supplies

20. **DC-DC 5V Regulator Circuit**
    - Can power host devices like Raspberry Pi
    - Research needed: specifications, current capacity, limitations

21. **Power Switch**
    - Controls external power supply
    - Research needed: specifications, current rating

### Motor Interfaces
22. **Motor Interface PH2.0 6P (Group A - with encoder)**
    - Interface for motor with encoder
    - Research needed: pin mappings, compatible motors, usage examples

23. **Motor Interface PH2.0 6P (Group B - with encoder)**
    - Interface for motor with encoder
    - Research needed: pin mappings, compatible motors, usage examples

24. **Motor Interface PH2.0 2P (Group A - without encoder)**
    - LED lamp interface in this product
    - Research needed: pin mappings, compatible devices, usage examples

25. **Motor Interface PH2.0 2P (Group B - without encoder)**
    - Interface for motor without encoder
    - Research needed: pin mappings, compatible motors, usage examples

## Software Components

26. **Web Interface**
    - Browser-based control system
    - Research needed: features, usage instructions, limitations

27. **JSON Command System**
    - Protocol for controlling the arm
    - Research needed: command structure, available commands, examples

28. **Python API**
    - Libraries for controlling the arm via Python
    - Research needed: installation, usage examples, capabilities

29. **ROS2 Integration**
    - Robot Operating System compatibility
    - Research needed: setup instructions, usage examples, capabilities

30. **LeRobot AI Project Integration**
    - AI capabilities for the robotic arm
    - Research needed: setup instructions, capabilities, examples

## Control Features

31. **AngleCtrl: Servo Angle Control**
    - Direct control of individual servo angles
    - Research needed: usage instructions, limitations, examples

32. **Torque Lock Control**
    - Control of servo torque locking
    - Research needed: usage instructions, applications, examples

33. **DEFA: Dynamic External Force Adaptive Control**
    - Adaptive control system
    - Research needed: usage instructions, applications, examples

34. **LED Light Control**
    - Control of LED lighting
    - Research needed: usage instructions, examples

35. **COORDCTRL: End Point Coordinate Control**
    - Inverse kinematics control system
    - Research needed: usage instructions, coordinate system, examples

36. **Leader-Follower Mode (Priority)**
    - Teaching by demonstration functionality
    - Research needed: setup instructions, usage examples, limitations
