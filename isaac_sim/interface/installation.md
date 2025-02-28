# Installing and Setting Up NVIDIA Isaac Sim for RoArm-M3 Pro

This guide provides step-by-step instructions for installing and configuring NVIDIA Isaac Sim to work with the RoArm-M3 Pro robotic arm.

## Prerequisites

Before installing Isaac Sim, ensure your system meets the following requirements:

### Hardware Requirements

- **GPU**: NVIDIA RTX GPU (RTX 2070 or better recommended)
- **CPU**: Intel Core i7 or AMD Ryzen 7 or better
- **RAM**: 32GB or more
- **Storage**: 50GB+ SSD space
- **OS**: Ubuntu 20.04 LTS or Windows 10/11

### Software Requirements

- **NVIDIA GPU Driver**: 525.60.13 or newer
- **CUDA Toolkit**: 11.8 or newer
- **Python**: 3.7 or newer
- **Git**: Latest version

## Installation Options

There are three ways to install NVIDIA Isaac Sim:

1. **Omniverse Launcher** (Recommended for beginners)
2. **Container Deployment** (Recommended for production)
3. **Python Package** (Recommended for development)

### Option 1: Omniverse Launcher Installation

1. **Download the Omniverse Launcher**:
   - Visit [NVIDIA Omniverse Download](https://www.nvidia.com/en-us/omniverse/download/)
   - Create an NVIDIA account if you don't have one
   - Download the appropriate launcher for your operating system

2. **Install the Launcher**:
   - Run the installer and follow the on-screen instructions
   - Sign in with your NVIDIA account

3. **Install Isaac Sim**:
   - In the Launcher, navigate to the "Exchange" tab
   - Find "Isaac Sim" and click "Install"
   - Follow the installation prompts

4. **Launch Isaac Sim**:
   - Once installed, navigate to the "Library" tab
   - Find Isaac Sim and click "Launch"

### Option 2: Container Deployment

1. **Install Docker**:
   ```bash
   # For Ubuntu
   sudo apt-get update
   sudo apt-get install docker.io nvidia-container-toolkit
   sudo systemctl restart docker
   ```

2. **Pull the Isaac Sim Container**:
   ```bash
   docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
   ```

3. **Run the Container**:
   ```bash
   docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v /etc/localtime:/etc/localtime:ro \
     -v ${HOME}/.Xauthority:/root/.Xauthority \
     -v ${HOME}/isaac-sim-workdir:/root/isaac-sim-workdir \
     -e DISPLAY=${DISPLAY} \
     nvcr.io/nvidia/isaac-sim:2023.1.1
   ```

4. **Launch Isaac Sim**:
   ```bash
   cd /isaac-sim && ./runheadless.native.sh
   ```

### Option 3: Python Package

1. **Install Miniconda**:
   ```bash
   wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
   bash Miniconda3-latest-Linux-x86_64.sh
   ```

2. **Create a Conda Environment**:
   ```bash
   conda create -n isaac-sim python=3.7
   conda activate isaac-sim
   ```

3. **Install Isaac Sim Python Package**:
   ```bash
   pip install nvidia-isaac-sim
   ```

4. **Verify Installation**:
   ```bash
   python -c "import omni.isaac.sim; print('Isaac Sim installed successfully')"
   ```

## Post-Installation Setup

After installing Isaac Sim, you need to configure it for use with the RoArm-M3 Pro:

### 1. Install Additional Python Packages

```bash
# Activate the Isaac Sim Python environment
cd ${ISAAC_SIM_PATH}
./python.sh -m pip install numpy scipy matplotlib pandas torch torchvision
```

### 2. Set Up the RoArm-M3 Pro Workspace

```bash
# Create a workspace directory
mkdir -p ~/isaac-sim-roarm
cd ~/isaac-sim-roarm

# Clone the RoArm-M3 repository (if available)
git clone https://github.com/jhacksman/RoArm-M3.git
```

### 3. Configure Environment Variables

Add the following to your `~/.bashrc` or `~/.zshrc` file:

```bash
# Isaac Sim Environment Variables
export ISAAC_SIM_PATH="/path/to/isaac-sim"  # Update with your installation path
export ISAAC_SIM_PYTHON="${ISAAC_SIM_PATH}/python.sh"
export PYTHONPATH="${PYTHONPATH}:${ISAAC_SIM_PATH}"
```

Source the updated configuration:

```bash
source ~/.bashrc  # or source ~/.zshrc
```

## Verifying the Installation

To verify that Isaac Sim is correctly installed and configured:

1. **Launch Isaac Sim**:
   ```bash
   cd ${ISAAC_SIM_PATH}
   ./isaac-sim.sh
   ```

2. **Run a Simple Test**:
   - In the Isaac Sim interface, go to "Window" > "Python Scripts"
   - Enter and run the following code:
   ```python
   import omni.isaac.core
   print("Isaac Sim is working correctly!")
   ```

## Troubleshooting

### Common Issues

1. **GPU Driver Issues**:
   ```bash
   # Check GPU driver version
   nvidia-smi
   
   # If driver is outdated, update it
   sudo apt-get update
   sudo apt-get install --only-upgrade nvidia-driver-525
   ```

2. **OpenGL Errors**:
   - Ensure your GPU supports OpenGL 4.6 or newer
   - Update your GPU drivers to the latest version

3. **Container Permission Issues**:
   ```bash
   # Add your user to the docker group
   sudo usermod -aG docker $USER
   # Log out and log back in for changes to take effect
   ```

4. **Python Package Conflicts**:
   - Use a dedicated conda environment for Isaac Sim
   - Avoid installing packages that might conflict with Isaac Sim dependencies

### Getting Help

If you encounter issues not covered in this guide:

- Check the [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
- Visit the [NVIDIA Developer Forums](https://forums.developer.nvidia.com/c/omniverse/isaac-sim/69)
- Search for solutions on [Stack Overflow](https://stackoverflow.com/questions/tagged/nvidia-isaac)

## Next Steps

After successfully installing and configuring Isaac Sim, proceed to:

1. [Creating a RoArm-M3 Pro Model](./model_creation.md)
2. [Setting up the Simulation Environment](./simulation_environment.md)
