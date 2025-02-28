# Installing LeRobot for RoArm-M3 Pro

This guide provides detailed instructions for installing and setting up the LeRobot framework for use with the RoArm-M3 Pro robotic arm.

## System Requirements

- **Operating System**: Windows 10/11, Ubuntu 20.04+, or macOS 10.15+
- **Python**: Python 3.8 or higher
- **Hardware**: 
  - RoArm-M3 Pro robotic arm
  - Computer with USB port and/or WiFi capability
  - Minimum 8GB RAM (16GB recommended for training)
  - NVIDIA GPU recommended for training (not required for inference)

## Installation Steps

### 1. Set Up Python Environment

It's recommended to use a virtual environment to avoid package conflicts:

#### For Windows:

```bash
# Install Python if not already installed
# Download from https://www.python.org/downloads/

# Create a virtual environment
python -m venv lerobot_env

# Activate the virtual environment
lerobot_env\Scripts\activate

# Upgrade pip
python -m pip install --upgrade pip
```

#### For Linux/macOS:

```bash
# Install Python if not already installed
# On Ubuntu: sudo apt install python3 python3-pip python3-venv

# Create a virtual environment
python3 -m venv lerobot_env

# Activate the virtual environment
source lerobot_env/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 2. Install LeRobot Framework

```bash
# Install LeRobot from PyPI
pip install lerobot

# Install additional dependencies
pip install numpy torch pyserial requests
```

For GPU support (recommended for training):

```bash
# Install PyTorch with CUDA support
pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu117
```

### 3. Install RoArm-M3 Python Package

```bash
# Clone the repository
git clone https://github.com/waveshare/RoArm-M3-Python.git

# Navigate to the directory
cd RoArm-M3-Python

# Install the package
pip install -e .
```

Alternatively, you can download the Python package from the Waveshare wiki:

1. Go to [Waveshare RoArm-M3 Wiki](https://www.waveshare.com/wiki/RoArm-M3)
2. Download the Python demo package (RoArm-M3_Python.zip)
3. Extract the package and install it:

```bash
unzip RoArm-M3_Python.zip
cd RoArm-M3_Python
pip install -e .
```

### 4. Install Additional Tools (Optional)

For visualization and debugging:

```bash
# Install visualization tools
pip install matplotlib opencv-python

# Install Jupyter for interactive development
pip install jupyter

# Install ROS2 bridge (if using ROS2)
# Follow ROS2 installation instructions: https://docs.ros.org/en/humble/Installation.html
```

## Verifying Installation

To verify that LeRobot and the RoArm-M3 Python package are installed correctly:

```python
# Create a test.py file
import lerobot
import roarm_m3

print(f"LeRobot version: {lerobot.__version__}")
print(f"RoArm-M3 package available: {'roarm_m3' in locals()}")

# Run the file
python test.py
```

## Testing Connection with RoArm-M3 Pro

To test the connection with your RoArm-M3 Pro:

1. Power on the RoArm-M3 Pro
2. Connect to the arm's WiFi (SSID: RoArm-M3, Password: 12345678)
3. Create and run the following script:

```python
# Create a connection_test.py file
import requests
import json

def test_connection():
    arm_ip = "192.168.4.1"
    cmd = {
        "type": "GetStatus"
    }
    url = f"http://{arm_ip}/js?json={json.dumps(cmd)}"
    try:
        response = requests.get(url, timeout=5)
        print(f"Connection successful: {response.text}")
        return True
    except Exception as e:
        print(f"Connection failed: {e}")
        return False

if __name__ == "__main__":
    test_connection()
```

Run the script:

```bash
python connection_test.py
```

## Troubleshooting Installation

### Common Issues

1. **Package Not Found Errors**
   - Ensure your virtual environment is activated
   - Try reinstalling the package: `pip install --force-reinstall lerobot`

2. **Version Conflicts**
   - Use a clean virtual environment
   - Specify package versions if needed: `pip install lerobot==X.Y.Z`

3. **GPU Not Detected**
   - Verify CUDA installation: `python -c "import torch; print(torch.cuda.is_available())"`
   - Install the correct PyTorch version for your CUDA version

4. **Connection Issues with RoArm-M3 Pro**
   - Verify WiFi connection to the arm
   - Check IP address (default: 192.168.4.1)
   - Try connecting via USB and serial communication

### Getting Help

If you encounter issues not covered here:

1. Check the [LeRobot GitHub repository](https://github.com/huggingface/lerobot) for known issues
2. Join the [Hugging Face Discord](https://huggingface.co/join/discord) for community support
3. Visit the [Waveshare forum](https://www.waveshare.com/forum/) for RoArm-M3 specific issues

## Next Steps

After successful installation:

1. Explore the [interface documentation](./README.md) to learn how to control the arm
2. Try the [example projects](../examples/) to understand LeRobot capabilities
3. Set up your own project using the provided templates
