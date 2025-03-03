# Hardware Component Resources

This directory contains local copies of resources for various hardware components used in the RoArm-M3 Pro. The resources are organized by component to make it easier to find specific files.

## Directory Structure

```
research/hardware/
├── communication/
│   ├── CP2102/
│   │   ├── drivers/
│   │   ├── datasheets/
│   │   └── README.md
│   ├── IPEX_Gen_1/
│   │   ├── drivers/
│   │   ├── datasheets/
│   │   └── README.md
│   └── ...
├── servos/
│   ├── ST3235/
│   │   ├── drivers/
│   │   ├── datasheets/
│   │   ├── 3d_models/
│   │   └── README.md
│   ├── ST3215/
│   │   ├── drivers/
│   │   ├── datasheets/
│   │   ├── 3d_models/
│   │   └── README.md
│   └── ...
├── sensors/
│   ├── QMI8658/
│   │   ├── drivers/
│   │   ├── datasheets/
│   │   └── README.md
│   ├── AK09918C/
│   │   ├── drivers/
│   │   ├── datasheets/
│   │   └── README.md
│   └── ...
└── main_control/
    ├── ESP32-WROOM-32/
    │   ├── drivers/
    │   ├── datasheets/
    │   └── README.md
    └── ...
```

## Component Directories

Each component has its own directory containing:

- `drivers/`: Driver software, utilities, and tools needed to interface with the component
- `datasheets/`: Technical documentation, specifications, and user manuals
- `3d_models/`: 3D model files for mechanical components (servos, etc.)
- `README.md`: Component-specific information and usage instructions

## Rationale

This component-specific organization provides several benefits:

1. **Improved Discoverability**: All resources related to a specific component are grouped together
2. **Local Availability**: Resources are stored locally to ensure availability even if external sources become inaccessible
3. **Consistent Structure**: Each component follows the same directory structure for predictability
4. **Proper Attribution**: Original source links are maintained alongside local copies
5. **Version Control**: Resources are tracked in the repository, ensuring everyone has access to the same files

## Usage

When documenting components, follow these guidelines:

1. Store all component-specific files in the appropriate component directory
2. Include both external links and local file references in documentation
3. Update the component README with relevant information
4. Use relative paths when linking to local resources

## Download Scripts

Two Python scripts are provided to help manage these resources:

1. `download_resources.py`: Downloads external resources referenced in the documentation
2. `update_links.py`: Updates documentation to include links to local resources

Run these scripts from the repository root directory:

```bash
python download_resources.py
python update_links.py
```

## Notes on External Resources

The external resources stored in this repository are local copies of files originally provided by Waveshare and other manufacturers. They are stored here to ensure availability even if the original sources become inaccessible. All original source links are maintained in the documentation for proper attribution.

## Download Date

These resources were initially organized and downloaded on March 3, 2025.
