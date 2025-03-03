# INA219 Voltage Current Monitoring Chip Resources

This directory contains local copies of resources for the INA219 Voltage Current Monitoring Chip used in the RoArm-M3 Pro.

## Specifications

- **Manufacturer**: Texas Instruments
- **Type**: Current/voltage monitor with I²C interface
- **Model**: INA219
- **Shunt Voltage Range**: ±40mV, ±80mV, ±160mV, or ±320mV (programmable)
- **Bus Voltage Range**: 0-26V (with 16V option for higher resolution)
- **Resolution**: 
  - Shunt Voltage: 10μV
  - Bus Voltage: 4mV
- **Interface**: I²C (up to 400kHz)
- **Supply Voltage**: 3.0V to 5.5V
- **Operating Temperature**: -40°C to +125°C
- **Package**: SOT-23 or SOIC-8
- **Power Consumption**: 1mA (typical)
- **Accuracy**: ±0.5% (typical)

## Contents

### Datasheets

- `INA219_Datasheet.pdf`: Technical specifications and documentation for the INA219 chip

### Drivers

- `INA219_Calibration_Utilities.txt`: Information about calibration utilities for the INA219 chip
- `INA219_Example_Code.txt`: Example code for interfacing with the INA219 chip

## Usage in RoArm-M3 Pro

The INA219 chip in the RoArm-M3 Pro is used for:

1. Monitoring power consumption of the robot arm
2. Measuring current draw of motors and servos
3. Detecting overload conditions
4. Battery voltage monitoring
5. Power management and optimization

## Integration with ESP32

The INA219 communicates with the ESP32 main controller via I²C interface. The sensor data is processed by the ESP32 to:

1. Monitor real-time power consumption
2. Implement safety features to prevent overcurrent
3. Optimize power usage based on load conditions
4. Provide diagnostic information

## Calibration

The INA219 requires calibration for accurate measurements. The calibration process involves:

1. Setting the calibration register based on the shunt resistor value
2. Configuring the bus and shunt voltage ranges
3. Setting the ADC resolution and conversion time

For a typical 0.1 ohm shunt resistor and 2A maximum current:
```
Current_LSB = 0.1 mA/bit (100uA/bit)
Cal = 0.04096 / (Current_LSB * RSHUNT)
Cal = 0.04096 / (0.0001 * 0.1) = 4096
```

## Notes

These files are local copies of resources originally provided by Texas Instruments and Waveshare. They are stored here to ensure availability even if the original sources become inaccessible.

## Original Sources

- [Texas Instruments INA219 Product Page](https://www.ti.com/product/INA219)
- [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3)

## Download Date

These resources were downloaded on March 3, 2025.
