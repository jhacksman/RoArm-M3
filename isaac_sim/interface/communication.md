# Communication Interface Between Isaac Sim and RoArm-M3 Pro

This guide explains how to establish communication between NVIDIA Isaac Sim and the physical RoArm-M3 Pro robotic arm, enabling sim-to-real transfer and real-time control.

## Overview

Effective communication between the simulation and the physical arm is essential for:

1. **Sim-to-Real Transfer**: Deploying algorithms developed in simulation to the real arm
2. **Real-to-Sim Data Flow**: Capturing real-world data for improving simulation fidelity
3. **Hybrid Control**: Using simulation for planning and the real arm for execution
4. **Digital Twin**: Synchronizing the virtual and physical arms

## Communication Methods

There are several methods to establish communication between Isaac Sim and the RoArm-M3 Pro:

### 1. HTTP Communication

The RoArm-M3 Pro provides a web interface with HTTP endpoints for control. This is the simplest method to integrate with Isaac Sim.

#### Implementation

```python
import requests
import json
import omni.isaac.core as isaac_core

# Function to send commands to the physical arm
def send_command_to_arm(command_dict):
    """Send a command to the physical RoArm-M3 Pro via HTTP.
    
    Args:
        command_dict (dict): Command in JSON format
    
    Returns:
        dict: Response from the arm
    """
    arm_ip = "192.168.4.1"  # Default IP when connected to arm's WiFi
    url = f"http://{arm_ip}/js?json={json.dumps(command_dict)}"
    
    try:
        response = requests.get(url, timeout=5)
        return json.loads(response.text)
    except Exception as e:
        print(f"Error communicating with arm: {e}")
        return None

# Example: Synchronize simulation joint positions with physical arm
def sync_sim_to_real(articulation, controller):
    """Synchronize simulation joint positions to the physical arm.
    
    Args:
        articulation: Isaac Sim articulation object
        controller: Isaac Sim articulation controller
    """
    # Get joint positions from simulation
    joint_positions = articulation.get_joint_positions()
    
    # Send positions to physical arm
    for i, position in enumerate(joint_positions):
        command = {
            "type": "AngleCtrl",
            "id": i + 1,  # Joint IDs start from 1
            "angle": float(position),
            "speed": 50
        }
        send_command_to_arm(command)
```

### 2. ROS2 Bridge

For more advanced integration, you can use ROS2 as a communication bridge between Isaac Sim and the RoArm-M3 Pro.

#### Prerequisites

- ROS2 installed on both the simulation computer and the arm's controller
- ROS2 bridge for Isaac Sim installed
- ROS2 support enabled on the RoArm-M3 Pro

#### Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import omni.isaac.ros2_bridge as ros2_bridge

# Initialize ROS2 bridge
ros2_bridge.initialize()

# Create ROS2 node
node = Node("isaac_sim_roarm_bridge")

# Create publishers and subscribers
joint_state_pub = node.create_publisher(
    JointState,
    "/roarm/joint_commands",
    10
)

joint_state_sub = node.create_subscription(
    JointState,
    "/roarm/joint_states",
    lambda msg: handle_joint_state(msg),
    10
)

def handle_joint_state(msg):
    """Handle joint state messages from the physical arm."""
    # Update simulation with real arm's joint positions
    joint_positions = msg.position
    
    # Apply to simulation
    controller.apply_action(joint_positions)

def send_joint_commands(articulation):
    """Send joint commands from simulation to physical arm."""
    # Get joint positions from simulation
    joint_positions = articulation.get_joint_positions()
    
    # Create ROS2 message
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"]
    msg.position = joint_positions.tolist()
    
    # Publish message
    joint_state_pub.publish(msg)
```

### 3. Python API Wrapper

Create a custom Python API wrapper that bridges Isaac Sim and the RoArm-M3 Pro's control interface.

#### Implementation

```python
class RoArmBridge:
    """Bridge between Isaac Sim and RoArm-M3 Pro."""
    
    def __init__(self, arm_ip="192.168.4.1", simulation_articulation=None):
        """Initialize the bridge.
        
        Args:
            arm_ip (str): IP address of the physical arm
            simulation_articulation: Isaac Sim articulation object
        """
        self.arm_ip = arm_ip
        self.sim_articulation = simulation_articulation
        self.controller = None
        
        if self.sim_articulation:
            self.controller = self.sim_articulation.get_articulation_controller()
    
    def connect(self):
        """Establish connection with the physical arm."""
        try:
            # Test connection
            response = self.send_command({"type": "GetStatus"})
            if response:
                print(f"Connected to RoArm-M3 Pro at {self.arm_ip}")
                return True
            else:
                print(f"Failed to connect to RoArm-M3 Pro at {self.arm_ip}")
                return False
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def send_command(self, command_dict):
        """Send a command to the physical arm.
        
        Args:
            command_dict (dict): Command in JSON format
        
        Returns:
            dict: Response from the arm
        """
        url = f"http://{self.arm_ip}/js?json={json.dumps(command_dict)}"
        
        try:
            response = requests.get(url, timeout=5)
            return json.loads(response.text)
        except Exception as e:
            print(f"Error communicating with arm: {e}")
            return None
    
    def sync_sim_to_real(self):
        """Synchronize simulation joint positions to the physical arm."""
        if not self.sim_articulation:
            print("No simulation articulation set")
            return
        
        # Get joint positions from simulation
        joint_positions = self.sim_articulation.get_joint_positions()
        
        # Send positions to physical arm
        for i, position in enumerate(joint_positions):
            command = {
                "type": "AngleCtrl",
                "id": i + 1,  # Joint IDs start from 1
                "angle": float(position),
                "speed": 50
            }
            self.send_command(command)
    
    def sync_real_to_sim(self):
        """Synchronize physical arm joint positions to the simulation."""
        # Get joint positions from physical arm
        response = self.send_command({"type": "GetJointPositions"})
        
        if response and "positions" in response:
            joint_positions = response["positions"]
            
            # Apply to simulation
            if self.controller:
                self.controller.apply_action(joint_positions)
```

## Data Formats

### Joint Position Format

When communicating joint positions:

- Joint angles are in radians
- Joint order: [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
- Gripper position ranges from 0.0 (closed) to 1.0 (open)

### Command Format

Commands to the RoArm-M3 Pro use the JSON format:

```json
{
  "type": "CommandType",
  "param1": value1,
  "param2": value2,
  ...
}
```

Common command types:

- `"AngleCtrl"`: Control individual joint angles
- `"CoordCtrl"`: Control end effector coordinates
- `"GetJointPositions"`: Get current joint positions
- `"GetStatus"`: Get arm status

## Latency Considerations

Communication latency can affect control performance:

1. **Measure Latency**: Monitor round-trip time for commands
2. **Buffering**: Implement command buffering for smoother control
3. **Prediction**: Use predictive models to compensate for latency
4. **Asynchronous Communication**: Use asynchronous communication for non-critical data

```python
import time
import asyncio

async def measure_latency(bridge, iterations=10):
    """Measure communication latency.
    
    Args:
        bridge: RoArmBridge instance
        iterations: Number of measurements
    
    Returns:
        float: Average latency in milliseconds
    """
    latencies = []
    
    for _ in range(iterations):
        start_time = time.time()
        response = bridge.send_command({"type": "GetStatus"})
        end_time = time.time()
        
        if response:
            latency = (end_time - start_time) * 1000  # Convert to ms
            latencies.append(latency)
        
        await asyncio.sleep(0.1)
    
    if latencies:
        avg_latency = sum(latencies) / len(latencies)
        print(f"Average latency: {avg_latency:.2f} ms")
        return avg_latency
    else:
        print("Failed to measure latency")
        return None
```

## Error Handling

Implement robust error handling for reliable communication:

```python
def send_command_with_retry(bridge, command, max_retries=3, retry_delay=0.5):
    """Send a command with retry logic.
    
    Args:
        bridge: RoArmBridge instance
        command: Command dictionary
        max_retries: Maximum number of retry attempts
        retry_delay: Delay between retries in seconds
    
    Returns:
        dict: Response from the arm or None if all retries fail
    """
    for attempt in range(max_retries + 1):
        try:
            response = bridge.send_command(command)
            if response:
                return response
        except Exception as e:
            print(f"Attempt {attempt + 1}/{max_retries + 1} failed: {e}")
        
        if attempt < max_retries:
            print(f"Retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)
    
    print(f"Command failed after {max_retries + 1} attempts")
    return None
```

## Synchronization Strategies

### 1. Time-Based Synchronization

```python
def time_synchronized_control(bridge, simulation_context, duration=10.0, frequency=10):
    """Run time-synchronized control between simulation and physical arm.
    
    Args:
        bridge: RoArmBridge instance
        simulation_context: Isaac Sim simulation context
        duration: Duration of synchronization in seconds
        frequency: Control frequency in Hz
    """
    start_time = time.time()
    period = 1.0 / frequency
    
    while time.time() - start_time < duration:
        cycle_start = time.time()
        
        # Step simulation
        simulation_context.step()
        
        # Synchronize simulation to real arm
        bridge.sync_sim_to_real()
        
        # Calculate sleep time to maintain frequency
        elapsed = time.time() - cycle_start
        sleep_time = max(0, period - elapsed)
        time.sleep(sleep_time)
```

### 2. Event-Based Synchronization

```python
def event_synchronized_control(bridge, simulation_context, event_queue):
    """Run event-synchronized control between simulation and physical arm.
    
    Args:
        bridge: RoArmBridge instance
        simulation_context: Isaac Sim simulation context
        event_queue: Queue for synchronization events
    """
    while True:
        # Wait for event
        event = event_queue.get()
        
        if event["type"] == "STOP":
            break
        
        if event["type"] == "SYNC":
            # Step simulation
            simulation_context.step()
            
            # Synchronize based on direction
            if event["direction"] == "SIM_TO_REAL":
                bridge.sync_sim_to_real()
            elif event["direction"] == "REAL_TO_SIM":
                bridge.sync_real_to_sim()
```

## Integration with LeRobot

To integrate with the LeRobot AI framework:

```python
from lerobot.interface import RoArmInterface
import omni.isaac.core as isaac_core

class IsaacSimLeRobotBridge:
    """Bridge between Isaac Sim, LeRobot, and RoArm-M3 Pro."""
    
    def __init__(self, arm_ip="192.168.4.1", simulation_articulation=None):
        """Initialize the bridge.
        
        Args:
            arm_ip (str): IP address of the physical arm
            simulation_articulation: Isaac Sim articulation object
        """
        self.arm_ip = arm_ip
        self.sim_articulation = simulation_articulation
        self.lerobot_interface = RoArmInterface(connection_type="http", address=arm_ip)
    
    def connect(self):
        """Establish connections."""
        # Connect to physical arm via LeRobot
        self.lerobot_interface.connect()
        
        # Verify connection
        if self.lerobot_interface.is_connected():
            print(f"Connected to RoArm-M3 Pro via LeRobot at {self.arm_ip}")
            return True
        else:
            print(f"Failed to connect to RoArm-M3 Pro via LeRobot at {self.arm_ip}")
            return False
    
    def train_in_sim_deploy_to_real(self, model_path):
        """Train a model in simulation and deploy to the real arm.
        
        Args:
            model_path (str): Path to save/load the trained model
        """
        # Train in simulation
        # ... (training code)
        
        # Deploy to real arm via LeRobot
        self.lerobot_interface.load_model(model_path)
        self.lerobot_interface.start_model_control()
```

## Troubleshooting

### Common Issues

1. **Connection Failures**:
   - Verify the arm's IP address
   - Check that the arm is powered on and connected to the same network
   - Ensure no firewall is blocking communication

2. **Command Timeouts**:
   - Increase timeout values for commands
   - Check network stability
   - Reduce command frequency

3. **Synchronization Drift**:
   - Implement periodic re-synchronization
   - Use feedback control to correct drift
   - Reduce control frequency if necessary

### Diagnostic Tools

```python
def run_diagnostics(bridge):
    """Run communication diagnostics.
    
    Args:
        bridge: RoArmBridge instance
    
    Returns:
        dict: Diagnostic results
    """
    results = {
        "connection": False,
        "latency": None,
        "joint_control": False,
        "status_query": False
    }
    
    # Test connection
    try:
        response = bridge.send_command({"type": "GetStatus"})
        results["connection"] = response is not None
        results["status_query"] = response is not None
    except Exception:
        pass
    
    # Test latency
    try:
        latency = asyncio.run(measure_latency(bridge, iterations=5))
        results["latency"] = latency
    except Exception:
        pass
    
    # Test joint control
    try:
        # Get current position
        response = bridge.send_command({"type": "GetJointPositions"})
        if response and "positions" in response:
            # Send the same position back (no movement)
            for i, position in enumerate(response["positions"]):
                command = {
                    "type": "AngleCtrl",
                    "id": i + 1,
                    "angle": position,
                    "speed": 10
                }
                result = bridge.send_command(command)
                if result:
                    results["joint_control"] = True
    except Exception:
        pass
    
    return results
```

## Next Steps

After establishing communication:

1. Proceed to [Sim-to-Real Transfer Learning](./sim_to_real.md)
2. Explore [Example Projects](../examples/README.md)
3. Learn about [Domain Randomization](./simulation_environment.md#domain-randomization)
