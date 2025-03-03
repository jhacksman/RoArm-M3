# QMI8658 6-axis Motion Sensor Resources

This directory contains local copies of resources for the QMI8658 6-axis Motion Sensor used in the RoArm-M3 Pro.

## Specifications

- **Manufacturer**: QST Corporation
- **Type**: 6-axis MEMS motion sensor (3-axis accelerometer + 3-axis gyroscope)
- **Model**: QMI8658A
- **Accelerometer Range**: ±2g/±4g/±8g/±16g (programmable)
- **Gyroscope Range**: ±16/±32/±64/±128/±256/±512/±1024/±2048 dps (programmable)
- **Resolution**: 16-bit ADC for both accelerometer and gyroscope
- **Interface**: I²C (up to 400kHz) and SPI (up to 10MHz)
- **Supply Voltage**: 1.71V to 3.6V
- **Operating Temperature**: -40°C to +85°C
- **Package**: 2.5mm × 3.0mm × 0.83mm LGA-14
- **Power Consumption**: 
  - Accelerometer: 130μA (normal mode)
  - Gyroscope: 1.1mA (normal mode)
  - Sleep mode: 3μA

## Contents

### Datasheets

- `QMI8658A_Datasheet.pdf`: Technical specifications and documentation for the QMI8658A sensor

### Drivers

- `QMI8658_Calibration_Tool.txt`: Information about calibration tools for the QMI8658 sensor
- `QMI8658_Firmware.txt`: Information about firmware for the QMI8658 sensor

## Usage in RoArm-M3 Pro

The QMI8658 sensor in the RoArm-M3 Pro is used for:

1. Motion detection and tracking
2. Orientation sensing
3. Vibration monitoring
4. Gesture recognition
5. Inertial navigation

## Integration with ESP32

The QMI8658 communicates with the ESP32 main controller via I²C interface. The sensor data is processed by the ESP32 to:

1. Determine the arm's orientation in space
2. Detect movements and gestures
3. Provide feedback for motion control algorithms
4. Enable leader-follower mode functionality

## Sample Code

Basic code to initialize and read from the QMI8658 sensor:

```c
#include <Wire.h>

#define QMI8658_ADDR 0x6B  // I2C address of QMI8658
#define WHO_AM_I     0x00  // Device ID register

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  // Check if sensor is connected
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(QMI8658_ADDR, 1);
  
  if (Wire.available()) {
    byte deviceID = Wire.read();
    Serial.print("Device ID: 0x");
    Serial.println(deviceID, HEX);
  }
  
  // Initialize sensor (simplified)
  // In a real application, you would configure various settings
}

void loop() {
  // Read accelerometer and gyroscope data
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(0x3A);  // Starting register for accel data
  Wire.endTransmission(false);
  Wire.requestFrom(QMI8658_ADDR, 12);  // Request 12 bytes (6 registers, 2 bytes each)
  
  if (Wire.available() >= 12) {
    int16_t ax = Wire.read() | (Wire.read() << 8);
    int16_t ay = Wire.read() | (Wire.read() << 8);
    int16_t az = Wire.read() | (Wire.read() << 8);
    int16_t gx = Wire.read() | (Wire.read() << 8);
    int16_t gy = Wire.read() | (Wire.read() << 8);
    int16_t gz = Wire.read() | (Wire.read() << 8);
    
    // Print values
    Serial.print("Accel: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.println(az);
    
    Serial.print("Gyro: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);
  }
  
  delay(100);
}
```

## Notes

These files are local copies of resources originally provided by QST Corporation and Waveshare. They are stored here to ensure availability even if the original sources become inaccessible.

## Original Sources

- [QST Corporation QMI8658 Product Page](https://www.qstcorp.com/en/product/detail/22)
- [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3)

## Download Date

These resources were downloaded on March 3, 2025.
