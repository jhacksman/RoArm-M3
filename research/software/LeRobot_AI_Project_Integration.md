# LeRobot AI Project Integration for RoArm-M3 Pro

## Overview

The RoArm-M3 Pro robotic arm is designed to integrate with the LeRobot AI project, a state-of-the-art machine learning framework for robotics developed by Hugging Face. This integration enables the RoArm-M3 Pro to leverage advanced artificial intelligence capabilities, including imitation learning, reinforcement learning, and deep learning techniques. By combining the hardware capabilities of the RoArm-M3 Pro with the software intelligence of LeRobot, users can develop sophisticated robotic applications that can learn from demonstration, adapt to new tasks, and perform complex manipulations.

![LeRobot AI Integration](https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/lerobot/lerobot-banner.png)

## Key Features

- **Pre-trained Models**: Access to LeRobot's pre-trained models for common robotic tasks
- **Teaching Datasets**: Utilize curated datasets for training robotic arm movements
- **Simulation Environments**: Test algorithms in simulation before deploying to the physical arm
- **Imitation Learning**: Train the arm to mimic demonstrated movements
- **Reinforcement Learning**: Develop policies that optimize for specific rewards
- **Deep Learning Integration**: Leverage neural networks for complex control tasks
- **Leader-Follower Capabilities**: Implement leader-follower mode using LeRobot's framework
- **Cross-Platform Support**: Works across different operating systems and hardware configurations

## LeRobot Framework Overview

LeRobot is an open-source framework that provides state-of-the-art machine learning tools for real-world robotics in PyTorch. It offers:

1. **Standardized Interfaces**: Common APIs for different robotic platforms
2. **Policy Learning Algorithms**: Implementation of modern policy learning approaches
3. **Data Collection Tools**: Utilities for gathering training data
4. **Simulation Integration**: Connections to physics engines like Genesis
5. **Deployment Utilities**: Tools for deploying trained models to physical robots

## Setting Up LeRobot with RoArm-M3 Pro

### Prerequisites

Before integrating LeRobot with the RoArm-M3 Pro, ensure you have:

1. **Python Environment**: Python 3.10 with required dependencies
2. **RoArm-M3 Pro**: The robotic arm with firmware supporting JSON commands
3. **Communication Interface**: WiFi connection or USB-Serial connection to the arm
4. **Development Environment**: A computer with sufficient processing power for ML tasks

### Installation Steps

1. **Create a Python Virtual Environment**:
   ```bash
   conda create -y -n lerobot python=3.10
   conda activate lerobot
   ```

2. **Install LeRobot**:
   ```bash
   git clone https://github.com/huggingface/lerobot.git
   cd lerobot
   pip install -e .
   ```

3. **Install Additional Dependencies**:
   ```bash
   # For visualization
   pip install matplotlib opencv-python
   
   # For communication with RoArm-M3 Pro
   pip install pyserial requests
   ```

4. **Download Pre-trained Models** (optional):
   ```bash
   # Example command to download a pre-trained model
   python -c "from huggingface_hub import snapshot_download; snapshot_download(repo_id='lerobot/act_so100_pick_place')"
   ```

## Data Collection for Training

LeRobot requires training data to learn new tasks. With the RoArm-M3 Pro, you can collect data in several ways:

### 1. Manual Demonstration

Record joint positions and end-effector coordinates while manually guiding the arm through desired movements:

1. **Enable Torque Off Mode**: Use the web interface or JSON command to disable torque
2. **Record Movements**: Use the provided Python script to record joint positions during manual guidance
3. **Process Data**: Convert the recorded data to LeRobot's format

### 2. Leader-Follower Mode

Use a second RoArm-M3 Pro as a leader to demonstrate movements:

1. **Set Up Leader Arm**: Configure one arm as the leader
2. **Set Up Follower Arm**: Configure the second arm as the follower
3. **Record Demonstrations**: Capture the movements and sensor data during demonstration

### 3. Teleoperation

Control the arm remotely and record the movements:

1. **Set Up Remote Control**: Use the web interface or custom application
2. **Perform Tasks**: Execute the desired tasks while recording
3. **Save Data**: Store the recorded trajectories in the appropriate format

### Data Format

LeRobot expects data in a specific format:

```
dataset_name/        # Task name
│
├── episode_0001     # First trajectory
│    ├──colors/      # Image information
│    │   ├──0.jpg
│    │   ├──1.jpg
│    │   └──...
│    ├──depths/      # Depth image information (if available)
│    │   ├──0.jpg
│    │   ├──1.jpg
│    │   └──...
│    └──data.json    # State and action information
├── episode_0002
├── episode_...
└── episode_xxxx
```

## Training Models with LeRobot

After collecting data, you can train models using LeRobot's training scripts:

### 1. Prepare the Dataset

Convert your collected data to LeRobot's format:

```bash
cd lerobot
python lerobot/scripts/push_dataset_to_hub.py \
    --raw-dir ~/datasets/roarm_m3_task \
    --raw-format json \
    --push-to-hub 0 \
    --repo-id RoArm_M3_Task \
    --local-dir ~/lerobot_datasets/RoArm_M3_Task \
    --fps 30
```

### 2. Train a Model

Train a model using one of LeRobot's policy learning algorithms:

#### Diffusion Policy

```bash
python lerobot/scripts/train.py \
    policy=diffusion_roarm_m3 \
    env=roarm_m3 \
    dataset_repo_id=RoArm_M3_Task
```

#### Action Chunking Transformer (ACT)

```bash
python lerobot/scripts/train.py \
    policy=act_roarm_m3 \
    env=roarm_m3 \
    dataset_repo_id=RoArm_M3_Task
```

### 3. Evaluate the Model

Test the trained model in simulation or on the physical robot:

```bash
python lerobot/scripts/eval.py \
    --pretrained-policy-name-or-path ~/lerobot/outputs/train/[timestamp]/checkpoints/[step]/pretrained_model
```

## Deploying Trained Models to RoArm-M3 Pro

After training, deploy the model to control the RoArm-M3 Pro:

### 1. HTTP Communication (WiFi)

```python
import rclpy
from rclpy.node import Node
import requests
import torch
from lerobot.policies import load_policy

class RoArmLeRobotNode(Node):
    def __init__(self):
        super().__init__('roarm_lerobot_node')
        self.ip_address = '192.168.4.1'  # Default IP in AP mode
        
        # Load the trained policy
        self.policy = load_policy("path/to/trained/model")
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Initialize state
        self.state = None
        
        self.get_logger().info('RoArm LeRobot node initialized')
    
    def control_loop(self):
        # Get current state (this would be from sensors, cameras, etc.)
        self.update_state()
        
        # Get action from policy
        with torch.no_grad():
            action = self.policy(self.state)
        
        # Convert action to JSON command
        json_cmd = self.action_to_json(action)
        
        # Send command to arm
        self.send_command(json_cmd)
    
    def update_state(self):
        # Get arm status
        status = self.send_command({"type": "GetStatus"})
        # Process status into state representation
        # ...
    
    def action_to_json(self, action):
        # Convert policy output to JSON command
        # Example:
        return {
            "type": "AngleCtrl",
            "id": 1,
            "angle": float(action[0]),
            "speed": 50
        }
    
    def send_command(self, json_cmd):
        try:
            url = f"http://{self.ip_address}/js?json={str(json_cmd)}"
            response = requests.get(url)
            return response.text
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = RoArmLeRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Serial Communication (USB)

```python
import serial
import threading
import json
import torch
from lerobot.policies import load_policy

class RoArmLeRobotController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        
        # Load the trained policy
        self.policy = load_policy("path/to/trained/model")
        
        # Initialize serial connection
        self.ser = serial.Serial(self.port, baudrate=self.baudrate, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        
        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Initialize state
        self.state = None
        
        print('RoArm LeRobot controller initialized')
    
    def read_serial(self):
        while True:
            try:
                if self.ser.in_waiting:
                    data = self.ser.readline().decode('utf-8').strip()
                    if data:
                        print(f'Received: {data}')
                        # Process feedback if needed
            except Exception as e:
                print(f'Error reading from serial: {e}')
    
    def control_loop(self):
        while True:
            try:
                # Get current state (this would be from sensors, cameras, etc.)
                self.update_state()
                
                # Get action from policy
                with torch.no_grad():
                    action = self.policy(self.state)
                
                # Convert action to JSON command
                json_cmd = self.action_to_json(action)
                
                # Send command to arm
                self.send_command(json_cmd)
                
                # Wait for next control cycle
                time.sleep(0.1)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f'Error in control loop: {e}')
    
    def update_state(self):
        # Get arm status
        self.send_command({"type": "GetStatus"})
        # Process status into state representation
        # ...
    
    def action_to_json(self, action):
        # Convert policy output to JSON command
        # Example:
        return {
            "type": "AngleCtrl",
            "id": 1,
            "angle": float(action[0]),
            "speed": 50
        }
    
    def send_command(self, json_cmd):
        try:
            cmd_str = json.dumps(json_cmd) + '\n'
            self.ser.write(cmd_str.encode())
            print(f'Command sent: {json_cmd}')
        except Exception as e:
            print(f'Error sending command: {e}')

if __name__ == '__main__':
    controller = RoArmLeRobotController()
    controller.control_loop()
```

## Example Use Cases

### 1. Pick and Place Task

Train the RoArm-M3 Pro to pick up objects from one location and place them in another:

1. **Collect Data**: Demonstrate the pick and place task multiple times
2. **Train Model**: Use the collected data to train a policy
3. **Deploy**: Run the trained model on the RoArm-M3 Pro

### 2. Object Sorting

Train the arm to sort objects by type, color, or size:

1. **Set Up Environment**: Arrange objects for sorting
2. **Collect Data**: Demonstrate sorting tasks
3. **Train Model**: Train a policy to recognize and sort objects
4. **Deploy**: Run the trained model on the RoArm-M3 Pro

### 3. Drawing or Writing

Train the arm to draw patterns or write text:

1. **Collect Data**: Guide the arm through drawing or writing motions
2. **Train Model**: Train a policy to reproduce the demonstrated patterns
3. **Deploy**: Run the trained model on the RoArm-M3 Pro

## Integration with Leader-Follower Mode

LeRobot's framework is particularly well-suited for implementing leader-follower mode with the RoArm-M3 Pro. This mode allows one arm (the leader) to guide the movements of another arm (the follower):

1. **Data Collection**: Use the leader arm to collect demonstration data
2. **Real-time Control**: Implement real-time control of the follower arm based on the leader's movements
3. **Learning from Demonstration**: Train models that can generalize from the leader's demonstrations

For detailed implementation of leader-follower mode, refer to the [Leader-Follower Mode documentation](../leader_follower_mode/README.md).

## Limitations and Considerations

When integrating the RoArm-M3 Pro with LeRobot, consider the following limitations:

1. **Computational Requirements**: Training deep learning models requires significant computational resources
2. **Data Quality**: The quality of the trained models depends on the quality and quantity of the collected data
3. **Real-time Performance**: The ESP32 has limited processing power for running complex models
4. **Sensor Limitations**: The available sensors on the RoArm-M3 Pro may limit the types of tasks that can be learned
5. **Communication Latency**: WiFi or serial communication may introduce latency in real-time control

## Available Resources

### Official Documentation
- [LeRobot GitHub Repository](https://github.com/huggingface/lerobot)
- [RoArm-M3 Wiki Page](https://www.waveshare.com/wiki/RoArm-M3)
- [Hugging Face LeRobot Documentation](https://huggingface.co/docs/lerobot)

### Community Resources
- [Unitree IL LeRobot](https://github.com/unitreerobotics/unitree_IL_lerobot) - Adaptation of LeRobot for Unitree robots
- [Slobot](https://github.com/alexis779/slobot) - LeRobot SO-ARM-100 integration with Genesis simulator

### Related Projects
- [Genesis Physics Engine](https://github.com/Genesis-Embodied-AI/Genesis) - Physics engine for robotics simulation
- [AVP Teleoperate](https://github.com/unitreerobotics/avp_teleoperate) - Teleoperation project for data collection

### Academic References
```
@misc{cadene2024lerobot,
    author = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Wolf, Thomas},
    title = {LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch},
    howpublished = "\url{https://github.com/huggingface/lerobot}",
    year = {2024}
}
```

## Conclusion

The integration of the RoArm-M3 Pro with the LeRobot AI project opens up a wide range of possibilities for advanced robotic applications. By leveraging LeRobot's pre-training models, teaching datasets, and simulation environments, users can develop sophisticated control systems that learn from demonstration and adapt to new tasks. This integration bridges the gap between hardware capabilities and software intelligence, enabling the RoArm-M3 Pro to perform complex tasks that would be difficult to program explicitly.

As both the RoArm-M3 Pro and LeRobot continue to evolve, this integration will likely become even more powerful, enabling increasingly sophisticated robotic applications in research, education, and industry.
