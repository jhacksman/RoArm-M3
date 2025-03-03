# ESP32-WROOM-32 Main Control Module Resources

This directory contains local copies of resources for the ESP32-WROOM-32 Main Control Module used in the RoArm-M3 Pro.

## Specifications

- **Manufacturer**: Espressif Systems
- **Type**: System-on-Chip (SoC) microcontroller
- **Model**: ESP32-WROOM-32
- **CPU**: Dual-core Xtensa LX6 microprocessor, up to 240 MHz
- **Memory**: 
  - 520 KB SRAM
  - 4 MB Flash (external)
- **Wireless Connectivity**:
  - Wi-Fi: 802.11 b/g/n (2.4 GHz)
  - Bluetooth: v4.2 BR/EDR and BLE
- **Peripherals**:
  - 34 programmable GPIO pins
  - 12-bit ADC (up to 18 channels)
  - 8-bit DAC (2 channels)
  - SPI, I2C, I2S, UART
  - PWM
  - Touch sensors
  - Hall sensor
- **Security**: Secure boot, Flash encryption
- **Operating Voltage**: 3.0V to 3.6V
- **Operating Temperature**: -40°C to +85°C
- **Package**: 18mm × 25.5mm × 3.1mm SMD module
- **Power Consumption**: 
  - Deep sleep: 10μA
  - Light sleep: 100μA
  - Active: ~240mA (peak)

## Contents

### Datasheets

- `ESP32-WROOM-32_Datasheet.pdf`: Technical specifications and documentation for the ESP32-WROOM-32 module
- `ESP32_Technical_Reference_Manual.pdf`: Detailed technical reference for the ESP32 chip
- `ESP-IDF_Programming_Guide.pdf`: Programming guide for the ESP-IDF framework

### Firmware

- `ESP32_Firmware_Tools.txt`: Information about firmware tools for the ESP32
- `ESP32_Example_Code.txt`: Example code for programming the ESP32

## Usage in RoArm-M3 Pro

The ESP32-WROOM-32 module in the RoArm-M3 Pro serves as the main controller and is responsible for:

1. Processing JSON commands received via Wi-Fi
2. Controlling servo motors
3. Reading sensor data
4. Implementing control algorithms
5. Managing wireless communication
6. Hosting the web interface
7. Implementing leader-follower mode functionality

## Development Environment

The ESP32 can be programmed using several development environments:

1. **Arduino IDE**
   - Easy to use for beginners
   - Requires ESP32 board support package
   - Limited access to advanced features

2. **ESP-IDF (Espressif IoT Development Framework)**
   - Official development framework from Espressif
   - Full access to all ESP32 features
   - Based on FreeRTOS
   - Steeper learning curve

3. **PlatformIO**
   - IDE extension for VSCode
   - Supports both Arduino framework and ESP-IDF
   - Better project management and dependency handling

## Firmware Update Process

To update the ESP32 firmware in the RoArm-M3 Pro:

1. Connect the arm to your computer via USB
2. Use esptool.py or ESP Flash Download Tool to flash the firmware
3. For OTA (Over-The-Air) updates, use the web interface or ESP-IDF OTA functionality

## Notes

These files are local copies of resources originally provided by Espressif Systems and Waveshare. They are stored here to ensure availability even if the original sources become inaccessible.

## Original Sources

- [Espressif ESP32-WROOM-32 Product Page](https://www.espressif.com/en/products/modules/esp32)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3)

## Download Date

These resources were downloaded on March 3, 2025.
