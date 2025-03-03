# QMI8658 6-axis Motion Sensor

## Overview

The QMI8658 is a 6-axis motion sensor (3-axis accelerometer and 3-axis gyroscope) manufactured by QST Corporation, integrated into the General Driver Board for Robots used in the RoArm-M3 Pro robotic arm. This high-precision MEMS (Micro-Electro-Mechanical Systems) inertial measurement unit provides motion detection and orientation data that, when combined with the AK09918C 3-axis electronic compass, forms a complete 9-axis IMU system for accurate orientation and position tracking of the robotic arm.

![QMI8658 Chip](https://www.waveshare.com/w/upload/c/c2/UGV01_tutorial_II01.png)

## Key Features

- **Complete 6D MEMS IMU**: Integrates 3-axis accelerometer and 3-axis gyroscope
- **High Precision**: Tight board-level gyroscope sensitivity of ±3%
- **Low Noise**: Gyroscope noise floor of 0.004 dps/√Hz
- **Wide Dynamic Range**: 
  - Accelerometer: ±2g/±4g/±8g/±16g selectable
  - Gyroscope: ±16/±32/±64/±128/±256/±512/±1024/±2048 dps selectable
- **High Resolution**: 16-bit ADC for both accelerometer and gyroscope
- **Low Power Consumption**: Multiple power modes for battery optimization
- **Multiple Interfaces**: I²C and SPI digital interfaces
- **Built-in Temperature Sensor**: For temperature compensation
- **Advanced Features**:
  - Programmable interrupts
  - FIFO buffer for data batching
  - Motion detection
  - Tap detection

## Technical Specifications

### Electrical Characteristics
- **Supply Voltage**: 1.71V to 3.6V
- **I/O Voltage**: 1.71V to 3.6V
- **Current Consumption**: 
  - 3.3mA in normal mode (both accel and gyro active)
  - Down to 7μA in low power mode
- **Operating Temperature Range**: -40°C to +85°C

### Accelerometer Specifications
- **Measurement Range**: ±2g/±4g/±8g/±16g (selectable)
- **Sensitivity**: 
  - 16,384 LSB/g (±2g range)
  - 8,192 LSB/g (±4g range)
  - 4,096 LSB/g (±8g range)
  - 2,048 LSB/g (±16g range)
- **Zero-g Offset**: ±40mg
- **Noise Density**: 100μg/√Hz
- **Output Data Rate**: 1.953Hz to 8000Hz

### Gyroscope Specifications
- **Measurement Range**: ±16/±32/±64/±128/±256/±512/±1024/±2048 dps (selectable)
- **Sensitivity**: 
  - 2,048 LSB/dps (±16 dps range)
  - 1,024 LSB/dps (±32 dps range)
  - 512 LSB/dps (±64 dps range)
  - 256 LSB/dps (±128 dps range)
  - 128 LSB/dps (±256 dps range)
  - 64 LSB/dps (±512 dps range)
  - 32 LSB/dps (±1024 dps range)
  - 16 LSB/dps (±2048 dps range)
- **Zero-Rate Offset**: ±3 dps
- **Noise Density**: 0.004 dps/√Hz
- **Output Data Rate**: 1.953Hz to 8000Hz

### Communication Interfaces
- **I²C**: Up to 400kHz
- **SPI**: Up to 10MHz
- **Register-Based**: Configuration and data registers accessible via I²C or SPI

### Physical Dimensions
- **Package**: LGA-14
- **Size**: 2.5mm × 3.0mm × 0.83mm

## Role in RoArm-M3 Pro

In the RoArm-M3 Pro robotic arm, the QMI8658 6-axis motion sensor is integrated into the General Driver Board for Robots and serves several critical functions:

1. **Motion Detection**: Detects and measures linear acceleration and angular velocity
2. **Orientation Tracking**: Provides data for calculating the robotic arm's orientation in space
3. **Vibration Monitoring**: Detects unwanted vibrations during operation
4. **9-Axis IMU Integration**: Works with the AK09918C 3-axis electronic compass to form a complete 9-axis IMU
5. **Stability Control**: Helps maintain stability during precise movements
6. **Feedback System**: Provides real-time feedback for control algorithms

The motion sensor data is particularly important for applications requiring precise orientation control, such as the leader-follower mode where the arm needs to accurately track and replicate movements.

## Programming and Integration

The QMI8658 in the RoArm-M3 Pro can be accessed and controlled through the ESP32 microcontroller using the I²C interface. Waveshare provides demo code and libraries for interfacing with the QMI8658 chip as part of the IMU system.

### I²C Communication

The QMI8658 uses a standard I²C interface with the following connections:
- **SCL**: Serial Clock Line (connected to ESP32 GPIO22)
- **SDA**: Serial Data Line (connected to ESP32 GPIO21)
- **VDD**: Power Supply (1.8V to 3.3V)
- **GND**: Ground

### Register Map

The QMI8658 contains several registers for configuration and data reading. Some key registers include:

| Register | Address | Description |
|----------|---------|-------------|
| WHO_AM_I | 0x00    | Device ID (0x05) |
| CTRL1    | 0x02    | Control register 1 (accelerometer configuration) |
| CTRL2    | 0x03    | Control register 2 (gyroscope configuration) |
| CTRL3    | 0x04    | Control register 3 (power mode) |
| CTRL7    | 0x08    | Control register 7 (data ready, FIFO) |
| AX_L     | 0x35    | Accelerometer X-axis low byte |
| AX_H     | 0x36    | Accelerometer X-axis high byte |
| AY_L     | 0x37    | Accelerometer Y-axis low byte |
| AY_H     | 0x38    | Accelerometer Y-axis high byte |
| AZ_L     | 0x39    | Accelerometer Z-axis low byte |
| AZ_H     | 0x3A    | Accelerometer Z-axis high byte |
| GX_L     | 0x3B    | Gyroscope X-axis low byte |
| GX_H     | 0x3C    | Gyroscope X-axis high byte |
| GY_L     | 0x3D    | Gyroscope Y-axis low byte |
| GY_H     | 0x3E    | Gyroscope Y-axis high byte |
| GZ_L     | 0x3F    | Gyroscope Z-axis low byte |
| GZ_H     | 0x40    | Gyroscope Z-axis high byte |
| TEMP_L   | 0x33    | Temperature low byte |
| TEMP_H   | 0x34    | Temperature high byte |

### Configuration Options

The QMI8658 supports multiple configuration options that can be set via the control registers:

1. **Accelerometer Range**: ±2g, ±4g, ±8g, or ±16g
2. **Gyroscope Range**: ±16, ±32, ±64, ±128, ±256, ±512, ±1024, or ±2048 dps
3. **Output Data Rate**: From 1.953Hz to 8000Hz
4. **Power Modes**: Normal, low power, or power down
5. **FIFO Configuration**: FIFO mode, watermark level, etc.
6. **Interrupt Configuration**: Data ready, motion detection, etc.

## Available Resources

### Documentation
- [QMI8658A Datasheet (Waveshare)](https://files.waveshare.com/upload/e/e7/QMI8658A.pdf) ([Local Copy](QMI8658/datasheets/QMI8658A.pdf))
- [General Driver for Robots Wiki Page](https://www.waveshare.com/wiki/General_Driver_for_Robots)
- [Tutorial VII: IMU Data Reading Demo](https://www.waveshare.com/wiki/Tutorial_V:_IMU_Data_Reading_Demo)

### Software
- [IMU Data Reading Demo](https://files.waveshare.com/wiki/common/9DOF_Demo.zip) ([Local Copy](QMI8658/drivers/9DOF_Demo.zip))
- [IMU Data Reading Demo v2](https://files.waveshare.com/upload/3/39/9DOF_Demo_v2.zip) ([Local Copy](QMI8658/drivers/9DOF_Demo_v2.zip))
- [QMI8658 Arduino Library by Seeed Studio](https://github.com/Seeed-Studio/Seeed_Arduino_QMI8658)
- [UGV01 Open-source Demo](https://files.waveshare.com/upload/0/0c/UGV01_Basic_Demo.zip) ([Local Copy](QMI8658/drivers/UGV01_Basic_Demo.zip)) (Contains IMU examples)

### Hardware Resources
- [Circuit Diagram](https://files.waveshare.com/upload/3/37/General_Driver_for_Robots_SCH.pdf) ([Local Copy](../main_control/ESP32-WROOM-32/datasheets/General_Driver_for_Robots_SCH.pdf))
- [General Driver for Robots STEP Model](https://files.waveshare.com/upload/8/8e/General_Driver_for_Robots_STEP.zip) ([Local Copy](../main_control/ESP32-WROOM-32/drivers/General_Driver_for_Robots_STEP.zip))

## Programming Examples

### Arduino Example (Basic IMU Reading)
```cpp
#include <Wire.h>

// QMI8658 I2C address
#define QMI8658_I2C_ADDR    0x6B

// QMI8658 Register addresses
#define QMI8658_WHO_AM_I    0x00
#define QMI8658_CTRL1       0x02
#define QMI8658_CTRL2       0x03
#define QMI8658_CTRL3       0x04
#define QMI8658_CTRL7       0x08
#define QMI8658_TEMP_L      0x33
#define QMI8658_TEMP_H      0x34
#define QMI8658_AX_L        0x35
#define QMI8658_AX_H        0x36
#define QMI8658_AY_L        0x37
#define QMI8658_AY_H        0x38
#define QMI8658_AZ_L        0x39
#define QMI8658_AZ_H        0x3A
#define QMI8658_GX_L        0x3B
#define QMI8658_GX_H        0x3C
#define QMI8658_GY_L        0x3D
#define QMI8658_GY_H        0x3E
#define QMI8658_GZ_L        0x3F
#define QMI8658_GZ_H        0x40

// Accelerometer ranges
#define QMI8658_ACC_RANGE_2G    0x00
#define QMI8658_ACC_RANGE_4G    0x01
#define QMI8658_ACC_RANGE_8G    0x02
#define QMI8658_ACC_RANGE_16G   0x03

// Gyroscope ranges
#define QMI8658_GYR_RANGE_16DPS    0x00
#define QMI8658_GYR_RANGE_32DPS    0x01
#define QMI8658_GYR_RANGE_64DPS    0x02
#define QMI8658_GYR_RANGE_128DPS   0x03
#define QMI8658_GYR_RANGE_256DPS   0x04
#define QMI8658_GYR_RANGE_512DPS   0x05
#define QMI8658_GYR_RANGE_1024DPS  0x06
#define QMI8658_GYR_RANGE_2048DPS  0x07

// Output data rates
#define QMI8658_ODR_1000HZ    0x03

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize QMI8658
  if (!initQMI8658()) {
    Serial.println("QMI8658 not found!");
    while (1);
  }
  
  Serial.println("QMI8658 initialized successfully");
  
  // Configure accelerometer and gyroscope
  configQMI8658(QMI8658_ACC_RANGE_8G, QMI8658_GYR_RANGE_512DPS, QMI8658_ODR_1000HZ);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  float temperature;
  
  // Read accelerometer, gyroscope, and temperature data
  if (readSensorData(&ax, &ay, &az, &gx, &gy, &gz, &temperature)) {
    // Convert raw values to physical units
    float acc_factor = 9.8f / 4096.0f;  // For ±8g range
    float gyr_factor = 1.0f / 64.0f;    // For ±512dps range
    
    float acc_x = ax * acc_factor;
    float acc_y = ay * acc_factor;
    float acc_z = az * acc_factor;
    
    float gyr_x = gx * gyr_factor;
    float gyr_y = gy * gyr_factor;
    float gyr_z = gz * gyr_factor;
    
    // Print accelerometer data (in m/s²)
    Serial.print("Accel: X=");
    Serial.print(acc_x, 2);
    Serial.print(" Y=");
    Serial.print(acc_y, 2);
    Serial.print(" Z=");
    Serial.print(acc_z, 2);
    Serial.println(" m/s²");
    
    // Print gyroscope data (in dps)
    Serial.print("Gyro: X=");
    Serial.print(gyr_x, 2);
    Serial.print(" Y=");
    Serial.print(gyr_y, 2);
    Serial.print(" Z=");
    Serial.print(gyr_z, 2);
    Serial.println(" dps");
    
    // Print temperature
    Serial.print("Temp: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");
    
    Serial.println();
  }
  
  delay(100);
}

bool initQMI8658() {
  uint8_t who_am_i = readRegister(QMI8658_WHO_AM_I);
  
  // Check device ID
  if (who_am_i != 0x05) {
    return false;
  }
  
  return true;
}

void configQMI8658(uint8_t acc_range, uint8_t gyr_range, uint8_t odr) {
  // Configure accelerometer
  writeRegister(QMI8658_CTRL1, (acc_range << 4) | odr);
  
  // Configure gyroscope
  writeRegister(QMI8658_CTRL2, (gyr_range << 4) | odr);
  
  // Set to normal mode
  writeRegister(QMI8658_CTRL3, 0x00);
  
  // Enable data ready interrupt
  writeRegister(QMI8658_CTRL7, 0x03);
}

bool readSensorData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, float *temp) {
  uint8_t data[14];
  
  // Read all data at once
  Wire.beginTransmission(QMI8658_I2C_ADDR);
  Wire.write(QMI8658_TEMP_L);
  Wire.endTransmission(false);
  
  Wire.requestFrom(QMI8658_I2C_ADDR, 14);
  if (Wire.available() < 14) {
    return false;
  }
  
  for (int i = 0; i < 14; i++) {
    data[i] = Wire.read();
  }
  
  // Convert temperature (0.0625°C/LSB)
  int16_t temp_raw = (int16_t)(data[1] << 8 | data[0]);
  *temp = temp_raw * 0.0625f;
  
  // Convert accelerometer data
  *ax = (int16_t)(data[3] << 8 | data[2]);
  *ay = (int16_t)(data[5] << 8 | data[4]);
  *az = (int16_t)(data[7] << 8 | data[6]);
  
  // Convert gyroscope data
  *gx = (int16_t)(data[9] << 8 | data[8]);
  *gy = (int16_t)(data[11] << 8 | data[10]);
  *gz = (int16_t)(data[13] << 8 | data[12]);
  
  return true;
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(QMI8658_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(QMI8658_I2C_ADDR, 1);
  return Wire.read();
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMI8658_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
```

### ESP32 Example for RoArm-M3 Pro (Using Waveshare Demo)
```cpp
#include <Arduino.h>
#include "IMU.h"

// Pin definitions for ESP32 on General Driver Board
#define SDA_PIN 21
#define SCL_PIN 22

// Variables for IMU data
int IMU_Roll = 0;
int IMU_Pitch = 0;
int IMU_Yaw = 0;
int IMU_Temp = 0;

// Structures for IMU data
IMU_ST_ANGLES_DATA stAngles;         // For storing angle data
IMU_ST_SENSOR_DATA stGyroRawData;    // For storing raw gyroscope data
IMU_ST_SENSOR_DATA stAccelRawData;   // For storing raw accelerometer data
IMU_ST_SENSOR_DATA stMagnRawData;    // For storing raw magnetometer data

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  // Initialize I2C with specific pins for ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  
  Serial.println("RoArm-M3 Pro 6-Axis Motion Sensor Test");
  
  // Initialize IMU
  if (imuInit()) {
    Serial.println("IMU initialized successfully");
  } else {
    Serial.println("Failed to initialize IMU");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  // Get IMU data
  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  IMU_Temp = QMI8658_readTemp();
  IMU_Roll = stAngles.fRoll;
  IMU_Pitch = stAngles.fPitch;
  IMU_Yaw = stAngles.fYaw;
  
  // Print angle data
  Serial.print("Roll: ");
  Serial.print(IMU_Roll);
  Serial.println(" degrees");
  
  Serial.print("Pitch: ");
  Serial.print(IMU_Pitch);
  Serial.println(" degrees");
  
  Serial.print("Yaw: ");
  Serial.print(IMU_Yaw);
  Serial.println(" degrees");
  
  Serial.print("Temperature: ");
  Serial.print(IMU_Temp);
  Serial.println(" °C");
  
  // Print raw accelerometer data
  Serial.print("Accel Raw: X=");
  Serial.print(stAccelRawData.s16X);
  Serial.print(" Y=");
  Serial.print(stAccelRawData.s16Y);
  Serial.print(" Z=");
  Serial.println(stAccelRawData.s16Z);
  
  // Print raw gyroscope data
  Serial.print("Gyro Raw: X=");
  Serial.print(stGyroRawData.s16X);
  Serial.print(" Y=");
  Serial.print(stGyroRawData.s16Y);
  Serial.print(" Z=");
  Serial.println(stGyroRawData.s16Z);
  
  Serial.println();
  delay(500);
}
```

## Troubleshooting

### Common Issues

1. **Inaccurate Readings**
   - Ensure proper calibration
   - Check for vibration or mechanical stress
   - Verify proper mounting orientation

2. **Communication Failures**
   - Check I²C connections
   - Verify correct I²C address (0x6B)
   - Ensure proper pull-up resistors on SDA/SCL lines

3. **Drift Issues**
   - Implement temperature compensation
   - Allow warm-up time for sensor stabilization
   - Use complementary or Kalman filtering

4. **Noise Problems**
   - Implement digital filtering
   - Ensure proper power supply filtering
   - Check for electromagnetic interference sources

## Calibration Process

For accurate motion sensing, the QMI8658 requires calibration to account for offset and sensitivity variations:

1. **Gyroscope Calibration**:
   - Keep the sensor stationary
   - Collect multiple samples (typically 100+)
   - Calculate the average offset for each axis
   - Subtract these offsets from raw readings

2. **Accelerometer Calibration**:
   - Position the sensor in 6 different orientations (±X, ±Y, ±Z facing up)
   - Collect data in each orientation
   - Calculate scaling factors and offsets
   - Apply corrections to raw readings

3. **Temperature Compensation**:
   - Collect sensor data across different temperatures
   - Create a temperature-to-offset mapping
   - Apply temperature-based corrections during operation

## Integration with Orientation Estimation

To calculate accurate orientation from the QMI8658 data:

1. **Complementary Filter**:
   - Combine accelerometer data (low-pass filtered) with integrated gyroscope data (high-pass filtered)
   - Simple and computationally efficient
   - Good for moderate dynamic conditions

2. **Kalman Filter**:
   - More complex but provides optimal estimation
   - Accounts for sensor noise characteristics
   - Better performance in highly dynamic conditions

3. **Madgwick or Mahony Filter**:
   - Specifically designed for IMU/AHRS applications
   - Good balance between accuracy and computational cost
   - Works well with magnetometer fusion (9-axis IMU)

## Conclusion

The QMI8658 6-axis Motion Sensor is a critical component in the RoArm-M3 Pro robotic arm, providing precise motion detection and orientation data as part of the 9-axis IMU system. Its high precision, low noise, and I²C interface make it ideal for integration with the ESP32-based control system, enabling accurate orientation tracking for the robotic arm's movements. When properly calibrated and used in conjunction with the AK09918C 3-axis electronic compass, it provides a complete solution for spatial orientation awareness in robotic applications.
