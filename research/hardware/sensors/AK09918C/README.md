# AK09918C 3-axis Electronic Compass Resources

This directory contains local copies of resources for the AK09918C 3-axis Electronic Compass used in the RoArm-M3 Pro.

## Specifications

- **Manufacturer**: AKM Semiconductor
- **Type**: 3-axis electronic compass (magnetometer)
- **Model**: AK09918C
- **Measurement Range**: ±4900 μT
- **Resolution**: 0.15 μT/LSB (16-bit)
- **Interface**: I²C (up to 400kHz)
- **Supply Voltage**: 1.65V to 3.6V
- **Operating Temperature**: -30°C to +85°C
- **Package**: 2.0mm × 2.0mm × 0.5mm WL-CSP
- **Power Consumption**: 
  - Measurement mode: 1.1mA
  - Standby mode: 0.9μA

## Contents

### Datasheets

- `AK09918C_Datasheet.pdf`: Technical specifications and documentation for the AK09918C sensor

### Drivers

- `AK09918C_Calibration_Software.txt`: Information about calibration software for the AK09918C sensor

## Usage in RoArm-M3 Pro

The AK09918C sensor in the RoArm-M3 Pro is used for:

1. Heading determination
2. Orientation sensing
3. Motion tracking when combined with accelerometer and gyroscope
4. Absolute position reference

## Integration with ESP32

The AK09918C communicates with the ESP32 main controller via I²C interface. The sensor data is processed by the ESP32 to:

1. Determine the arm's orientation relative to magnetic north
2. Provide absolute heading reference for navigation
3. Compensate for gyroscope drift
4. Enable more accurate motion tracking

## Sample Code

Basic code to initialize and read from the AK09918C sensor:

```c
#include <Wire.h>

#define AK09918C_ADDR 0x0C  // I2C address of AK09918C
#define WHO_AM_I     0x00   // Device ID register
#define WIA_VAL      0x48   // Expected WHO_AM_I value
#define MODE_REG     0x31   // Control register
#define MODE_SNG     0x01   // Single measurement mode
#define ST1_REG      0x10   // Status register 1
#define DATA_READY   0x01   // Data ready bit
#define HXL_REG      0x11   // X-axis data register (low byte)

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  // Check if sensor is connected
  Wire.beginTransmission(AK09918C_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(AK09918C_ADDR, 2);
  
  if (Wire.available() >= 2) {
    byte deviceID = Wire.read();
    Serial.print("Device ID: 0x");
    Serial.println(deviceID, HEX);
    
    if (deviceID == WIA_VAL) {
      Serial.println("AK09918C found!");
    } else {
      Serial.println("AK09918C not found!");
    }
  }
}

void loop() {
  // Set to single measurement mode
  Wire.beginTransmission(AK09918C_ADDR);
  Wire.write(MODE_REG);
  Wire.write(MODE_SNG);
  Wire.endTransmission();
  
  // Wait for data ready
  bool dataReady = false;
  for (int i = 0; i < 10; i++) {
    delay(10);
    Wire.beginTransmission(AK09918C_ADDR);
    Wire.write(ST1_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AK09918C_ADDR, 1);
    
    if (Wire.available()) {
      if (Wire.read() & DATA_READY) {
        dataReady = true;
        break;
      }
    }
  }
  
  if (dataReady) {
    // Read magnetometer data
    Wire.beginTransmission(AK09918C_ADDR);
    Wire.write(HXL_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AK09918C_ADDR, 6);
    
    if (Wire.available() >= 6) {
      int16_t mx = Wire.read() | (Wire.read() << 8);
      int16_t my = Wire.read() | (Wire.read() << 8);
      int16_t mz = Wire.read() | (Wire.read() << 8);
      
      // Print values
      Serial.print("Mag: ");
      Serial.print(mx); Serial.print(", ");
      Serial.print(my); Serial.print(", ");
      Serial.println(mz);
      
      // Calculate heading (simplified, no tilt compensation)
      float heading = atan2(my, mx) * 180.0 / PI;
      if (heading < 0) heading += 360.0;
      
      Serial.print("Heading: ");
      Serial.println(heading);
    }
  }
  
  delay(100);
}
```

## Notes

These files are local copies of resources originally provided by AKM Semiconductor and Waveshare. They are stored here to ensure availability even if the original sources become inaccessible.

## Original Sources

- [AKM Semiconductor AK09918C Product Page](https://www.akm.com/global/en/products/electronic-compass/)
- [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3)

## Download Date

These resources were downloaded on March 3, 2025.
