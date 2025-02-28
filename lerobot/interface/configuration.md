# LeRobot Configuration for RoArm-M3 Pro

This document provides detailed information on configuring LeRobot to work optimally with the RoArm-M3 Pro robotic arm. Proper configuration ensures efficient communication, accurate control, and effective learning.

## Configuration Overview

LeRobot configuration for the RoArm-M3 Pro involves several components:

1. **Communication Configuration**: How LeRobot communicates with the arm
2. **Robot Model Configuration**: Defining the arm's physical properties
3. **Learning Configuration**: Settings for data collection and model training
4. **Deployment Configuration**: Parameters for deploying trained models

## Communication Configuration

### HTTP Communication

Configure LeRobot to communicate with the RoArm-M3 Pro via HTTP:

```python
# config.py
HTTP_CONFIG = {
    "ip_address": "192.168.4.1",  # Default IP when connected to arm's WiFi
    "port": 80,                   # Default HTTP port
    "timeout": 5,                 # Request timeout in seconds
    "retry_attempts": 3,          # Number of retry attempts
    "retry_delay": 0.5            # Delay between retries in seconds
}
```

Usage:

```python
from config import HTTP_CONFIG
import requests
import json
import time

def send_command(command):
    url = f"http://{HTTP_CONFIG['ip_address']}/js?json={json.dumps(command)}"
    for attempt in range(HTTP_CONFIG['retry_attempts']):
        try:
            response = requests.get(url, timeout=HTTP_CONFIG['timeout'])
            return response.text
        except Exception as e:
            print(f"Attempt {attempt+1} failed: {e}")
            if attempt < HTTP_CONFIG['retry_attempts'] - 1:
                time.sleep(HTTP_CONFIG['retry_delay'])
    return None
```

### Serial Communication

Configure LeRobot to communicate with the RoArm-M3 Pro via serial connection:

```python
# config.py
SERIAL_CONFIG = {
    "port": "/dev/ttyUSB0",  # Windows: "COM3", adjust as needed
    "baudrate": 115200,       # Default baudrate
    "timeout": 1,             # Read timeout in seconds
    "write_timeout": 1,       # Write timeout in seconds
    "retry_attempts": 3,      # Number of retry attempts
    "retry_delay": 0.5        # Delay between retries in seconds
}
```

Usage:

```python
from config import SERIAL_CONFIG
import serial
import json
import time

def send_serial_command(command):
    for attempt in range(SERIAL_CONFIG['retry_attempts']):
        try:
            with serial.Serial(
                SERIAL_CONFIG['port'],
                baudrate=SERIAL_CONFIG['baudrate'],
                timeout=SERIAL_CONFIG['timeout'],
                write_timeout=SERIAL_CONFIG['write_timeout']
            ) as ser:
                cmd_str = json.dumps(command) + '\n'
                ser.write(cmd_str.encode())
                time.sleep(0.1)  # Give time for processing
                response = ser.readline().decode().strip()
                return response
        except Exception as e:
            print(f"Attempt {attempt+1} failed: {e}")
            if attempt < SERIAL_CONFIG['retry_attempts'] - 1:
                time.sleep(SERIAL_CONFIG['retry_delay'])
    return None
```

## Robot Model Configuration

Define the RoArm-M3 Pro's physical properties for LeRobot:

```python
# config.py
ROARM_M3_CONFIG = {
    "joints": [
        {
            "id": 1,
            "name": "base",
            "type": "revolute",
            "limits": [-3.14, 3.14],  # Radians
            "default_position": 0.0
        },
        {
            "id": 2,
            "name": "shoulder",
            "type": "revolute",
            "limits": [-1.57, 1.57],  # Radians
            "default_position": 0.0
        },
        {
            "id": 3,
            "name": "elbow",
            "type": "revolute",
            "limits": [-1.57, 1.57],  # Radians
            "default_position": 0.0
        },
        {
            "id": 4,
            "name": "wrist_pitch",
            "type": "revolute",
            "limits": [-1.57, 1.57],  # Radians
            "default_position": 0.0
        },
        {
            "id": 5,
            "name": "wrist_roll",
            "type": "revolute",
            "limits": [-3.14, 3.14],  # Radians
            "default_position": 0.0
        },
        {
            "id": 6,
            "name": "gripper",
            "type": "prismatic",
            "limits": [0.0, 1.0],     # Normalized (0: closed, 1: open)
            "default_position": 0.5
        }
    ],
    "kinematics": {
        "dh_parameters": [
            # [a, alpha, d, theta]
            [0, 1.5708, 0.0715, 0],      # Joint 1
            [0.1175, 0, 0, 1.5708],      # Joint 2
            [0.096, 0, 0, 0],            # Joint 3
            [0, 1.5708, 0.0945, 0],      # Joint 4
            [0, 0, 0.0815, 0],           # Joint 5
            [0, 0, 0.05, 0]              # Joint 6
        ]
    },
    "max_velocity": [2.0, 1.5, 1.5, 2.0, 2.0, 1.0],  # rad/s for each joint
    "max_acceleration": [4.0, 3.0, 3.0, 4.0, 4.0, 2.0]  # rad/s² for each joint
}
```

Usage:

```python
from config import ROARM_M3_CONFIG
import numpy as np

def forward_kinematics(joint_angles):
    # Simplified FK calculation using DH parameters
    # In a real application, use a proper robotics library
    
    dh = ROARM_M3_CONFIG['kinematics']['dh_parameters']
    T = np.eye(4)
    
    for i, theta in enumerate(joint_angles):
        a, alpha, d, theta_offset = dh[i]
        theta = theta + theta_offset
        
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T_i = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        
        T = T @ T_i
    
    return T
```

## Learning Configuration

Configure LeRobot's learning parameters for the RoArm-M3 Pro:

```python
# config.py
LEARNING_CONFIG = {
    "data_collection": {
        "frequency": 10,           # Hz
        "duration": 300,           # seconds
        "save_directory": "./data",
        "include_sensors": True,
        "include_images": True,
        "camera_id": 0
    },
    "training": {
        "model_type": "imitation",  # "imitation", "reinforcement", "hybrid"
        "batch_size": 64,
        "learning_rate": 0.001,
        "epochs": 100,
        "validation_split": 0.2,
        "early_stopping": True,
        "patience": 10,
        "save_directory": "./models"
    },
    "augmentation": {
        "enabled": True,
        "position_noise": 0.05,    # radians
        "velocity_noise": 0.1,     # rad/s
        "random_crop": True,       # for image data
        "random_flip": True        # for image data
    }
}
```

Usage:

```python
from config import LEARNING_CONFIG
import os
import time
import numpy as np

def collect_data(arm, controller):
    # Create save directory if it doesn't exist
    save_dir = LEARNING_CONFIG['data_collection']['save_directory']
    os.makedirs(save_dir, exist_ok=True)
    
    # Set up data collection
    frequency = LEARNING_CONFIG['data_collection']['frequency']
    duration = LEARNING_CONFIG['data_collection']['duration']
    total_samples = frequency * duration
    
    # Initialize data arrays
    joint_positions = np.zeros((total_samples, len(ROARM_M3_CONFIG['joints'])))
    timestamps = np.zeros(total_samples)
    
    # Collect data
    print(f"Collecting data for {duration} seconds at {frequency} Hz...")
    for i in range(total_samples):
        # Get current joint positions
        joint_positions[i] = arm.get_joint_positions()
        timestamps[i] = time.time()
        
        # Control the arm (e.g., with a controller or manually)
        controller.step()
        
        # Sleep to maintain frequency
        time.sleep(1/frequency)
    
    # Save data
    np.save(f"{save_dir}/joint_positions.npy", joint_positions)
    np.save(f"{save_dir}/timestamps.npy", timestamps)
    
    print(f"Data collection complete. Saved to {save_dir}")
    return joint_positions, timestamps
```

## Deployment Configuration

Configure parameters for deploying trained models to the RoArm-M3 Pro:

```python
# config.py
DEPLOYMENT_CONFIG = {
    "inference": {
        "frequency": 10,           # Hz
        "model_path": "./models/latest.pt",
        "use_gpu": True,
        "optimize_model": True,    # Apply optimization techniques
        "batch_inference": False   # Real-time control requires single-sample inference
    },
    "safety": {
        "velocity_limit_factor": 0.8,  # Reduce max velocity to 80%
        "acceleration_limit_factor": 0.8,
        "emergency_stop_enabled": True,
        "collision_detection": True,
        "workspace_limits_enabled": True,
        "workspace_margin": 0.05   # meters
    },
    "logging": {
        "enabled": True,
        "level": "INFO",           # "DEBUG", "INFO", "WARNING", "ERROR"
        "log_file": "./logs/deployment.log",
        "console_output": True
    }
}
```

Usage:

```python
from config import DEPLOYMENT_CONFIG, ROARM_M3_CONFIG
import torch
import time
import logging
import os

def setup_logging():
    if DEPLOYMENT_CONFIG['logging']['enabled']:
        os.makedirs(os.path.dirname(DEPLOYMENT_CONFIG['logging']['log_file']), exist_ok=True)
        logging.basicConfig(
            level=getattr(logging, DEPLOYMENT_CONFIG['logging']['level']),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(DEPLOYMENT_CONFIG['logging']['log_file']),
                logging.StreamHandler() if DEPLOYMENT_CONFIG['logging']['console_output'] else logging.NullHandler()
            ]
        )

def deploy_model(arm):
    # Set up logging
    setup_logging()
    logger = logging.getLogger("deployment")
    
    # Load model
    logger.info(f"Loading model from {DEPLOYMENT_CONFIG['inference']['model_path']}")
    device = torch.device("cuda" if DEPLOYMENT_CONFIG['inference']['use_gpu'] and torch.cuda.is_available() else "cpu")
    model = torch.load(DEPLOYMENT_CONFIG['inference']['model_path'], map_location=device)
    
    if DEPLOYMENT_CONFIG['inference']['optimize_model']:
        logger.info("Optimizing model for inference")
        model = torch.jit.script(model)
    
    # Set up safety limits
    velocity_limits = np.array(ROARM_M3_CONFIG['max_velocity']) * DEPLOYMENT_CONFIG['safety']['velocity_limit_factor']
    acceleration_limits = np.array(ROARM_M3_CONFIG['max_acceleration']) * DEPLOYMENT_CONFIG['safety']['acceleration_limit_factor']
    
    logger.info("Starting control loop")
    
    # Control loop
    try:
        while True:
            start_time = time.time()
            
            # Get current state
            current_state = arm.get_state()
            
            # Predict next action
            with torch.no_grad():
                action = model(torch.FloatTensor(current_state).unsqueeze(0))
                action = action.squeeze(0).cpu().numpy()
            
            # Apply safety limits
            if DEPLOYMENT_CONFIG['safety']['velocity_limit_factor'] < 1.0:
                action = np.clip(action, -velocity_limits, velocity_limits)
            
            # Execute action
            arm.execute_action(action)
            
            # Sleep to maintain control frequency
            elapsed_time = time.time() - start_time
            sleep_time = max(0, (1.0 / DEPLOYMENT_CONFIG['inference']['frequency']) - elapsed_time)
            time.sleep(sleep_time)
            
            logger.debug(f"Control cycle time: {time.time() - start_time:.4f}s")
    
    except KeyboardInterrupt:
        logger.info("Control interrupted by user")
    except Exception as e:
        logger.error(f"Error during control: {e}")
    finally:
        logger.info("Stopping arm")
        arm.stop()
```

## ROS2 Configuration

If using ROS2 as a bridge between LeRobot and the RoArm-M3 Pro:

```python
# config.py
ROS2_CONFIG = {
    "node_name": "lerobot_roarm_bridge",
    "joint_state_topic": "/roarm_m3/joint_states",
    "joint_command_topic": "/roarm_m3/joint_commands",
    "gripper_command_topic": "/roarm_m3/gripper_command",
    "status_topic": "/roarm_m3/status",
    "namespace": "roarm_m3",
    "use_sim_time": False,
    "qos_profile": "sensor_data"  # Options: "sensor_data", "services", "parameters", "default"
}
```

Usage:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from config import ROS2_CONFIG

class RoArmM3Bridge(Node):
    def __init__(self):
        super().__init__(ROS2_CONFIG['node_name'])
        
        # Publishers
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            ROS2_CONFIG['joint_command_topic'],
            10
        )
        
        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            ROS2_CONFIG['joint_state_topic'],
            self.joint_state_callback,
            10
        )
        
        # Initialize arm connection
        self.arm = RoArmM3Connection()
        self.get_logger().info("RoArm-M3 Bridge initialized")
    
    def joint_state_callback(self, msg):
        # Process joint state messages from ROS2
        joint_positions = msg.position
        self.get_logger().debug(f"Received joint positions: {joint_positions}")
        
        # Forward to LeRobot
        # Implementation depends on LeRobot API
    
    def send_joint_command(self, joint_positions):
        # Create message
        msg = Float64MultiArray()
        msg.data = joint_positions
        
        # Publish command
        self.joint_command_publisher.publish(msg)
        self.get_logger().debug(f"Published joint command: {joint_positions}")

def main():
    rclpy.init()
    node = RoArmM3Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## NVIDIA Isaac Sim Configuration

For integration with NVIDIA Isaac Sim:

```python
# config.py
ISAAC_SIM_CONFIG = {
    "usd_path": "./assets/roarm_m3_pro.usd",
    "physics_dt": 1.0/60.0,
    "rendering_dt": 1.0/60.0,
    "stage_units_in_meters": 1.0,
    "robot_position": [0.0, 0.0, 0.0],
    "robot_orientation": [1.0, 0.0, 0.0, 0.0],  # Quaternion (w, x, y, z)
    "camera_position": [1.0, 1.0, 0.5],
    "camera_target": [0.0, 0.0, 0.2],
    "domain_randomization": {
        "enabled": True,
        "visual_randomization": True,
        "dynamics_randomization": True,
        "randomize_every_n_frames": 100
    }
}
```

Usage with NVIDIA Isaac Sim:

```python
import omni.isaac.core as isaac_core
import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.robots as robots
from config import ISAAC_SIM_CONFIG

def setup_isaac_sim():
    # Initialize Isaac Sim
    isaac_core.initialize()
    
    # Create a new stage
    stage_utils.create_new_stage()
    
    # Set physics properties
    physics_context = isaac_core.PhysicsContext()
    physics_context.set_gravity([0, 0, -9.81])
    physics_context.set_physics_dt(ISAAC_SIM_CONFIG['physics_dt'])
    physics_context.set_rendering_dt(ISAAC_SIM_CONFIG['rendering_dt'])
    
    # Load RoArm-M3 Pro model
    arm = robots.Robot(
        prim_path="/World/RoArmM3Pro",
        name="roarm_m3_pro",
        usd_path=ISAAC_SIM_CONFIG['usd_path']
    )
    
    # Set robot position and orientation
    arm.set_world_pose(
        position=ISAAC_SIM_CONFIG['robot_position'],
        orientation=ISAAC_SIM_CONFIG['robot_orientation']
    )
    
    # Set up camera
    camera = isaac_core.Camera(
        position=ISAAC_SIM_CONFIG['camera_position'],
        target=ISAAC_SIM_CONFIG['camera_target']
    )
    
    # Apply domain randomization if enabled
    if ISAAC_SIM_CONFIG['domain_randomization']['enabled']:
        domain_randomizer = isaac_core.DomainRandomizer()
        
        if ISAAC_SIM_CONFIG['domain_randomization']['visual_randomization']:
            domain_randomizer.add_visual_randomization(arm)
        
        if ISAAC_SIM_CONFIG['domain_randomization']['dynamics_randomization']:
            domain_randomizer.add_dynamics_randomization(arm)
    
    return arm, camera
```

## Conclusion

Proper configuration is essential for successful integration of the RoArm-M3 Pro with LeRobot. The configuration files and examples provided in this document serve as a starting point for your own implementation. Adjust the parameters based on your specific requirements and hardware setup.

For more information, refer to:
- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [RoArm-M3 Pro Wiki](https://www.waveshare.com/wiki/RoArm-M3)
- [JSON Command System Documentation](../research/software/JSON_Command_System.md)
- [ROS2 Integration Documentation](../research/software/ROS2_Integration.md)
