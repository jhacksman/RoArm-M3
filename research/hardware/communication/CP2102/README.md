# CP2102 Serial-to-USB Chip Resources

This directory contains local copies of resources for the CP2102 Serial-to-USB Chip used in the RoArm-M3 Pro.

## Contents

### Drivers

- `CP210x_Windows_Drivers.zip`: Windows drivers for the CP2102 chip
- `CP210x_MacOS_Drivers.dmg`: macOS drivers for the CP2102 chip
- `CP210x_Linux_Drivers.tar.gz`: Linux drivers for the CP2102 chip

### Datasheets

- `CP2102_Datasheet.pdf`: Technical specifications and documentation for the CP2102 chip

## Usage

1. Install the appropriate driver for your operating system
2. Connect the RoArm-M3 Pro to your computer via USB
3. The CP2102 chip should be recognized as a virtual COM port

## Serial Communication Parameters

When communicating with the RoArm-M3 Pro through the CP2102:

- **Baud Rate**: 115200 bps (default)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

## Notes

These files are local copies of resources originally provided by Waveshare and Silicon Labs. They are stored here to ensure availability even if the original sources become inaccessible.

## Original Sources

- [Waveshare Wiki CP2102 Drivers](https://files.waveshare.com/wiki/common/CP210x_Windows_Drivers.zip) ([Local Copy](drivers/CP210x_Windows_Drivers.zip))
- [Silicon Labs CP2102 Documentation](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- [CP2102 Datasheet (Silicon Labs)](https://www.silabs.com/documents/public/data-sheets/CP2102-9.pdf)

## Download Date

These resources were downloaded on March 3, 2025.
