# AK09918C 3-axis Electronic Compass

## Overview

The AK09918C is a 3-axis electronic compass IC manufactured by Asahi Kasei Microdevices (AKM), integrated into the General Driver Board for Robots used in the RoArm-M3 Pro robotic arm. This high-precision magnetic sensor provides heading information that, when combined with the QMI8658 6-axis motion sensor, forms a complete 9-axis IMU (Inertial Measurement Unit) system for accurate orientation and position tracking of the robotic arm.

![AK09918C Chip](https://www.akm.com/content/dam/documents/products/electronic-compass/ak09918c/ak09918c-thumbnail-1.jpg)

## Key Features

- **Ultra-Small Size**: 0.76mm × 0.76mm × 0.5mm (63% size of the conventional product AK09916C)
- **High Sensitivity**: Utilizes high-sensitive Hall sensor technology
- **Low Power Consumption**: Optimized for battery-powered applications
- **Wide Dynamic Range**: ±4900μT measurement range
- **High Resolution**: 0.15μT/LSB (16-bit resolution)
- **I²C Interface**: Standard communication protocol compatible with various microcontrollers
- **Magnetic Sensor Overflow Monitor**: Detects magnetic field anomalies
- **Internal Clock Source**: Built-in oscillator for stable operation
- **Multiple Operation Modes**: Power-down, single measurement, and continuous measurement modes
- **Small Package**: 4-pin WL-CSP (Wafer Level Chip Scale Package)

## Technical Specifications

### Electrical Characteristics
- **Supply Voltage**: 1.65V to 1.95V
- **Logic Interface Voltage**: 1.65V to 3.6V
- **Current Consumption**: 
  - 3μA in power-down mode
  - 1.1mA in active mode (100Hz measurement)
- **Operating Temperature Range**: -30°C to +85°C
- **Measurement Range**: ±4900μT (X, Y, Z axes)
- **Resolution**: 16-bit (0.15μT/LSB)
- **Noise**: 0.5μT RMS

### Communication Interface
- **Protocol**: I²C (Fast Mode: 400kHz)
- **Slave Address**: 0x0C (fixed)
- **Register-Based**: Configuration and data registers accessible via I²C

### Physical Dimensions
- **Package**: WL-CSP (Wafer Level Chip Scale Package)
- **Size**: 0.76mm × 0.76mm × 0.5mm
- **Pin Configuration**: 4-pin (2 for power supply, 2 for I²C communication)

## Role in RoArm-M3 Pro

In the RoArm-M3 Pro robotic arm, the AK09918C electronic compass is integrated into the General Driver Board for Robots and serves several critical functions:

1. **Heading Reference**: Provides absolute heading information for orientation control
2. **Magnetic Field Detection**: Measures the Earth's magnetic field for compass functionality
3. **9-Axis IMU Integration**: Works with the QMI8658 6-axis motion sensor to form a complete 9-axis IMU
4. **Orientation Tracking**: Enables accurate tracking of the robotic arm's orientation in space
5. **Calibration Reference**: Provides a reference for calibrating the robotic arm's movements

The compass data is particularly important for applications requiring precise orientation control, such as the leader-follower mode where the arm needs to maintain awareness of its spatial orientation.

## Programming and Integration

The AK09918C in the RoArm-M3 Pro can be accessed and controlled through the ESP32 microcontroller using the I²C interface. Waveshare provides demo code and libraries for interfacing with the AK09918C chip as part of the IMU system.

### I²C Communication

The AK09918C uses a standard I²C interface with the following connections:
- **SCL**: Serial Clock Line (connected to ESP32 GPIO22)
- **SDA**: Serial Data Line (connected to ESP32 GPIO21)
- **VDD**: Power Supply (1.8V typical)
- **GND**: Ground

### Register Map

The AK09918C contains several registers for configuration and data reading:

| Register | Address | Description |
|----------|---------|-------------|
| WIA1     | 0x00    | Device ID (0x48) |
| WIA2     | 0x01    | Device ID (0x09) |
| ST1      | 0x10    | Status 1 (data status) |
| HXL      | 0x11    | X-axis measurement data (low byte) |
| HXH      | 0x12    | X-axis measurement data (high byte) |
| HYL      | 0x13    | Y-axis measurement data (low byte) |
| HYH      | 0x14    | Y-axis measurement data (high byte) |
| HZL      | 0x15    | Z-axis measurement data (low byte) |
| HZH      | 0x16    | Z-axis measurement data (high byte) |
| ST2      | 0x18    | Status 2 (data status) |
| CNTL1    | 0x30    | Control 1 (operation mode) |
| CNTL2    | 0x31    | Control 2 (soft reset) |
| CNTL3    | 0x32    | Control 3 (low power mode) |

### Operation Modes

The AK09918C supports multiple operation modes that can be set via the CNTL1 register:

| Mode | Value | Description |
|------|-------|-------------|
| Power-down | 0x00 | Power-down mode (default) |
| Single measurement | 0x01 | Single measurement mode |
| Continuous 1 | 0x02 | Continuous measurement mode (10Hz) |
| Continuous 2 | 0x04 | Continuous measurement mode (20Hz) |
| Continuous 3 | 0x06 | Continuous measurement mode (50Hz) |
| Continuous 4 | 0x08 | Continuous measurement mode (100Hz) |

## Available Resources

### Documentation
- [AK09918C Product Page (AKM)](https://www.akm.com/global/en/products/electronic-compass/ak09918c/)
- [General Driver for Robots Wiki Page](https://www.waveshare.com/wiki/General_Driver_for_Robots)
- [Tutorial VII: IMU Data Reading Demo](https://www.waveshare.com/wiki/Tutorial_V:_IMU_Data_Reading_Demo)

### Software
- [UGV01 Open-source Demo](https://files.waveshare.com/upload/0/0c/UGV01_Basic_Demo.zip) (Contains IMU examples)
- [QMI8658C and AK09918C Arduino Library](https://github.com/Seeed-Studio/Seeed_Arduino_QMI8658)
- [AK09918 Arduino Library by Creativite](https://github.com/Creativite-Tecnologia/AK09918)

### Hardware Resources
- [Circuit Diagram](https://files.waveshare.com/upload/3/37/General_Driver_for_Robots_SCH.pdf)
- [General Driver for Robots STEP Model](https://files.waveshare.com/upload/8/8e/General_Driver_for_Robots_STEP.zip)

## Programming Examples

### Arduino Example (Basic Compass Reading)
```cpp
#include <Wire.h>

// AK09918 I2C address
#define AK09918_I2C_ADDR    0x0C

// AK09918 Register addresses
#define AK09918_WIA1        0x00
#define AK09918_WIA2        0x01
#define AK09918_ST1         0x10
#define AK09918_HXL         0x11
#define AK09918_HXH         0x12
#define AK09918_HYL         0x13
#define AK09918_HYH         0x14
#define AK09918_HZL         0x15
#define AK09918_HZH         0x16
#define AK09918_ST2         0x18
#define AK09918_CNTL1       0x30
#define AK09918_CNTL2       0x31
#define AK09918_CNTL3       0x32

// AK09918 Operation modes
#define AK09918_POWER_DOWN           0x00
#define AK09918_SINGLE_MEASUREMENT   0x01
#define AK09918_CONTINUOUS_1         0x02
#define AK09918_CONTINUOUS_2         0x04
#define AK09918_CONTINUOUS_3         0x06
#define AK09918_CONTINUOUS_4         0x08

// Data ready bit
#define AK09918_DRDY_BIT             0x01

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize AK09918
  if (!initAK09918()) {
    Serial.println("AK09918 not found!");
    while (1);
  }
  
  Serial.println("AK09918 initialized successfully");
  
  // Set to continuous measurement mode (100Hz)
  writeRegister(AK09918_CNTL2, 0x01);  // Soft reset
  delay(100);
  writeRegister(AK09918_CNTL1, AK09918_CONTINUOUS_4);
}

void loop() {
  int16_t x, y, z;
  
  // Read magnetic data
  if (readMagneticData(&x, &y, &z)) {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Z: ");
    Serial.println(z);
    
    // Calculate heading (simplified)
    float heading = atan2(y, x) * 180.0 / PI;
    if (heading < 0) {
      heading += 360.0;
    }
    
    Serial.print("Heading: ");
    Serial.println(heading);
  }
  
  delay(100);
}

bool initAK09918() {
  uint8_t wia1, wia2;
  
  // Read WHO_AM_I registers
  wia1 = readRegister(AK09918_WIA1);
  wia2 = readRegister(AK09918_WIA2);
  
  // Check device ID
  if (wia1 != 0x48 || wia2 != 0x09) {
    return false;
  }
  
  return true;
}

bool readMagneticData(int16_t *x, int16_t *y, int16_t *z) {
  uint8_t status;
  uint8_t data[6];
  
  // Check if data is ready
  status = readRegister(AK09918_ST1);
  if (!(status & AK09918_DRDY_BIT)) {
    return false;
  }
  
  // Read all data at once
  Wire.beginTransmission(AK09918_I2C_ADDR);
  Wire.write(AK09918_HXL);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AK09918_I2C_ADDR, 6);
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }
  
  // Check for magnetic sensor overflow
  status = readRegister(AK09918_ST2);
  if (status & 0x08) {
    return false;  // Magnetic sensor overflow
  }
  
  // Convert data
  *x = (int16_t)(data[1] << 8 | data[0]);
  *y = (int16_t)(data[3] << 8 | data[2]);
  *z = (int16_t)(data[5] << 8 | data[4]);
  
  return true;
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(AK09918_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AK09918_I2C_ADDR, 1);
  return Wire.read();
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AK09918_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
```

### ESP32 Example for RoArm-M3 Pro (Using Seeed Library)
```cpp
#include <Arduino.h>
#include <Wire.h>
#include "QMI8658.h"
#include "AK09918.h"

// Pin definitions for ESP32 on General Driver Board
#define SDA_PIN 21
#define SCL_PIN 22

QMI8658 qmi;
AK09918 ak09918;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  // Initialize I2C with specific pins for ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  
  Serial.println("RoArm-M3 Pro 9-Axis IMU Test");
  
  // Initialize QMI8658 (accelerometer and gyroscope)
  if (qmi.begin()) {
    Serial.println("QMI8658 initialized successfully");
  } else {
    Serial.println("Failed to initialize QMI8658");
    while (1) {
      delay(10);
    }
  }
  
  // Initialize AK09918 (magnetometer)
  if (ak09918.initialize() == AK09918_ERR_OK) {
    Serial.println("AK09918 initialized successfully");
  } else {
    Serial.println("Failed to initialize AK09918");
    while (1) {
      delay(10);
    }
  }
  
  // Set QMI8658 configuration
  qmi.configAccel(QMI8658AccRange_8g, QMI8658AccOdr_1000Hz);
  qmi.configGyro(QMI8658GyrRange_512dps, QMI8658GyrOdr_1000Hz);
  
  // Set AK09918 to continuous measurement mode
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  int32_t mx, my, mz;
  
  // Read accelerometer and gyroscope data
  qmi.readAccel(&ax, &ay, &az);
  qmi.readGyro(&gx, &gy, &gz);
  
  // Read magnetometer data
  ak09918.getData(&mx, &my, &mz);
  
  // Print accelerometer data (in g)
  Serial.print("Accel: X=");
  Serial.print(ax, 2);
  Serial.print(" Y=");
  Serial.print(ay, 2);
  Serial.print(" Z=");
  Serial.print(az, 2);
  Serial.println(" g");
  
  // Print gyroscope data (in dps)
  Serial.print("Gyro: X=");
  Serial.print(gx, 2);
  Serial.print(" Y=");
  Serial.print(gy, 2);
  Serial.print(" Z=");
  Serial.print(gz, 2);
  Serial.println(" dps");
  
  // Print magnetometer data (in μT)
  Serial.print("Mag: X=");
  Serial.print(mx * 0.15); // Convert to μT (0.15 μT/LSB)
  Serial.print(" Y=");
  Serial.print(my * 0.15);
  Serial.print(" Z=");
  Serial.print(mz * 0.15);
  Serial.println(" μT");
  
  // Calculate heading (simplified)
  float heading = atan2(my, mx) * 180.0 / PI;
  if (heading < 0) {
    heading += 360.0;
  }
  
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.println(" degrees");
  
  Serial.println();
  delay(500);
}
```

## Troubleshooting

### Common Issues

1. **Inaccurate Readings**
   - Ensure proper calibration
   - Keep away from magnetic interference sources
   - Verify proper mounting orientation

2. **Communication Failures**
   - Check I²C connections
   - Verify correct I²C address (0x0C)
   - Ensure proper pull-up resistors on SDA/SCL lines

3. **Magnetic Interference**
   - Keep away from motors, speakers, and other magnetic sources
   - Consider magnetic shielding if necessary
   - Implement software filtering for noisy environments

4. **Calibration Issues**
   - Implement hard-iron and soft-iron calibration
   - Perform figure-8 motion for calibration
   - Store calibration values in non-volatile memory

## Calibration Process

For accurate heading information, the AK09918C requires calibration to account for hard-iron and soft-iron distortions:

1. **Hard-Iron Calibration**: Compensates for permanent magnetic fields
   - Collect data while rotating the device in all orientations
   - Calculate the center offset of the magnetic field sphere
   - Subtract these offsets from raw readings

2. **Soft-Iron Calibration**: Compensates for magnetic field distortions
   - Collect data while rotating the device in all orientations
   - Calculate the transformation matrix to convert the distorted ellipsoid to a sphere
   - Apply the transformation matrix to calibrated readings

## Conclusion

The AK09918C 3-axis Electronic Compass is a critical component in the RoArm-M3 Pro robotic arm, providing precise heading information as part of the 9-axis IMU system. Its small size, high sensitivity, and I²C interface make it ideal for integration with the ESP32-based control system, enabling accurate orientation tracking for the robotic arm's movements. When properly calibrated and used in conjunction with the QMI8658 6-axis motion sensor, it provides a complete solution for spatial orientation awareness in robotic applications.
