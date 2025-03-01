#!/usr/bin/env python3
"""
Sim-to-Real Calibration Example for RoArm-M3 Pro

This example demonstrates how to calibrate the simulation parameters
to match the behavior of the physical RoArm-M3 Pro robotic arm.
"""

import os
import time
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.articulations import ArticulationView

# Configuration
ARM_USD_PATH = "/path/to/roarm_m3.usd"  # Update with your USD path
SIMULATION_TIMESTEP = 1.0 / 60.0  # 60 Hz physics simulation
ARM_IP = "192.168.4.1"  # Default IP when connected to arm's WiFi
DATA_DIR = "/path/to/data"  # Directory to store calibration data

class RoArmBridge:
    """Bridge between Isaac Sim and the physical RoArm-M3 Pro."""
    
    def __init__(self, arm_ip=ARM_IP):
        """Initialize the bridge.
        
        Args:
            arm_ip (str): IP address of the physical arm
        """
        self.arm_ip = arm_ip
    
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
        import requests
        
        url = f"http://{self.arm_ip}/js?json={json.dumps(command_dict)}"
        
        try:
            response = requests.get(url, timeout=5)
            return json.loads(response.text)
        except Exception as e:
            print(f"Error communicating with arm: {e}")
            return None
    
    def get_joint_positions(self):
        """Get current joint positions from the physical arm.
        
        Returns:
            list: Joint positions in radians
        """
        response = self.send_command({"type": "GetJointPositions"})
        if response and "positions" in response:
            return response["positions"]
        return None
    
    def set_joint_positions(self, positions, speed=50):
        """Set joint positions on the physical arm.
        
        Args:
            positions (list): Target joint positions in radians
            speed (int): Movement speed (1-100)
        
        Returns:
            bool: Success flag
        """
        success = True
        for i, position in enumerate(positions):
            command = {
                "type": "AngleCtrl",
                "id": i + 1,  # Joint IDs start from 1
                "angle": float(position),
                "speed": speed
            }
            response = self.send_command(command)
            if not response:
                success = False
        
        return success

def setup_scene():
    """Set up the simulation scene."""
    # Create a new stage
    stage_utils.create_new_stage()
    
    # Set up the physics scene
    stage_utils.set_stage_up_axis("Z")
    
    # Create ground plane
    ground_prim = prim_utils.create_prim(
        "/World/ground",
        "Plane",
        attributes={"size": 10.0}
    )
    
    # Add physics properties to ground
    prim_utils.set_rigid_body_properties(
        ground_prim.GetPath(),
        mass=0,  # Static object
        linear_damping=0,
        angular_damping=0
    )
    
    # Add lighting
    dome_light = prim_utils.create_prim(
        "/World/Lights/DomeLight",
        "DomeLight"
    )
    prim_utils.set_attribute(dome_light.GetPath(), "intensity", 1000.0)
    
    # Import arm model
    if os.path.exists(ARM_USD_PATH):
        from omni.kit.commands import execute
        execute("CreateReferenceCommand",
            asset_path=ARM_USD_PATH,
            prim_path="/World/RoArm_M3"
        )
        print(f"Imported RoArm-M3 Pro model from {ARM_USD_PATH}")
        return True
    else:
        print(f"USD model not found at {ARM_USD_PATH}")
        print("Creating simplified arm model instead")
        create_simplified_arm()
        return False

def create_simplified_arm():
    """Create a simplified RoArm-M3 Pro model."""
    # Create base
    base_prim = prim_utils.create_prim(
        "/World/RoArm_M3/base",
        "Cylinder",
        attributes={"radius": 0.1, "height": 0.05}
    )
    
    # Create articulation root
    prim_utils.set_articulation_root_properties(base_prim.GetPath())
    
    # Create joints and links (simplified for this example)
    # In a real implementation, you would create a more accurate model
    # based on the RoArm-M3 Pro specifications
    
    # Create first arm segment
    arm1_prim = prim_utils.create_prim(
        "/World/RoArm_M3/arm1",
        "Cube",
        attributes={"size": (0.05, 0.05, 0.2)}
    )
    prim_utils.set_attribute(arm1_prim.GetPath(), "xformOp:translate", (0, 0, 0.15))
    
    # Create first joint
    joint1_prim = prim_utils.create_prim(
        "/World/RoArm_M3/base/joint1",
        "PhysicsJoint"
    )
    prim_utils.set_attribute(joint1_prim.GetPath(), "type", "revolute")
    prim_utils.set_attribute(joint1_prim.GetPath(), "body0", "/World/RoArm_M3/base")
    prim_utils.set_attribute(joint1_prim.GetPath(), "body1", "/World/RoArm_M3/arm1")
    prim_utils.set_attribute(joint1_prim.GetPath(), "axis", (0, 0, 1))
    prim_utils.set_attribute(joint1_prim.GetPath(), "lowerLimit", -3.14)
    prim_utils.set_attribute(joint1_prim.GetPath(), "upperLimit", 3.14)
    
    # Add more joints and links as needed for a complete model

def generate_test_trajectories(num_joints, num_trajectories=5, points_per_trajectory=20):
    """Generate test trajectories for calibration.
    
    Args:
        num_joints (int): Number of joints
        num_trajectories (int): Number of trajectories to generate
        points_per_trajectory (int): Number of points per trajectory
    
    Returns:
        list: List of trajectories, each containing joint positions
    """
    trajectories = []
    
    for _ in range(num_trajectories):
        trajectory = []
        
        # Generate random start and end positions
        start_positions = np.random.uniform(-0.5, 0.5, num_joints)
        end_positions = np.random.uniform(-0.5, 0.5, num_joints)
        
        # Generate trajectory points
        for i in range(points_per_trajectory):
            alpha = i / (points_per_trajectory - 1)
            positions = start_positions * (1 - alpha) + end_positions * alpha
            trajectory.append(positions.tolist())
        
        trajectories.append(trajectory)
    
    return trajectories

def collect_real_data(bridge, trajectories):
    """Collect data from the physical robot.
    
    Args:
        bridge (RoArmBridge): Communication bridge to the physical robot
        trajectories (list): List of trajectories to execute
    
    Returns:
        dict: Collected data
    """
    print("Collecting data from the physical RoArm-M3 Pro...")
    
    data = {
        "trajectories": trajectories,
        "joint_positions": [],
        "timestamps": []
    }
    
    # Connect to the physical arm
    if not bridge.connect():
        print("Failed to connect to the physical arm")
        return None
    
    # Execute each trajectory
    for traj_idx, trajectory in enumerate(trajectories):
        print(f"Executing trajectory {traj_idx+1}/{len(trajectories)}")
        
        for point_idx, joint_positions in enumerate(trajectory):
            print(f"  Point {point_idx+1}/{len(trajectory)}")
            
            # Send command to physical robot
            success = bridge.set_joint_positions(joint_positions, speed=30)
            if not success:
                print("  Failed to set joint positions")
                continue
            
            # Wait for movement to complete
            time.sleep(0.5)
            
            # Record data
            actual_positions = bridge.get_joint_positions()
            if actual_positions:
                data["joint_positions"].append(actual_positions)
                data["timestamps"].append(time.time())
            else:
                print("  Failed to get joint positions")
    
    print(f"Collected {len(data['joint_positions'])} data points")
    
    return data

def collect_sim_data(articulation, controller, trajectories, params=None):
    """Collect data from the simulation.
    
    Args:
        articulation (ArticulationView): Isaac Sim articulation object
        controller: Articulation controller
        trajectories (list): List of trajectories to execute
        params (dict, optional): Simulation parameters to apply
    
    Returns:
        dict: Collected data
    """
    print("Collecting data from the simulation...")
    
    data = {
        "trajectories": trajectories,
        "joint_positions": [],
        "timestamps": []
    }
    
    # Apply parameters if provided
    if params:
        apply_parameters_to_sim(articulation, params)
    
    # Create simulation context
    sim_context = SimulationContext.instance()
    
    # Execute each trajectory
    for traj_idx, trajectory in enumerate(trajectories):
        print(f"Executing trajectory {traj_idx+1}/{len(trajectories)}")
        
        for point_idx, joint_positions in enumerate(trajectory):
            print(f"  Point {point_idx+1}/{len(trajectory)}")
            
            # Send command to simulated robot
            controller.apply_action(joint_positions)
            
            # Step simulation
            for _ in range(30):  # 30 steps at 60Hz = 0.5s
                sim_context.step()
            
            # Record data
            data["joint_positions"].append(articulation.get_joint_positions().tolist())
            data["timestamps"].append(sim_context.current_time)
    
    print(f"Collected {len(data['joint_positions'])} data points")
    
    return data

def apply_parameters_to_sim(articulation, params):
    """Apply parameters to the simulation.
    
    Args:
        articulation (ArticulationView): Isaac Sim articulation object
        params (dict): Parameter dictionary
    """
    print("Applying parameters to simulation:")
    
    # Apply mass scales
    if "mass_scales" in params:
        for i, scale in enumerate(params["mass_scales"]):
            if i < articulation.num_links:
                current_mass = articulation.get_link_mass(i)
                articulation.set_link_mass(i, current_mass * scale)
                print(f"  Link {i} mass scale: {scale:.4f}")
    
    # Apply damping scales
    if "damping_scales" in params:
        for i, scale in enumerate(params["damping_scales"]):
            if i < articulation.num_joints:
                current_damping = articulation.get_joint_damping(i)
                articulation.set_joint_damping(i, current_damping * scale)
                print(f"  Joint {i} damping scale: {scale:.4f}")
    
    # Apply stiffness scales
    if "stiffness_scales" in params:
        for i, scale in enumerate(params["stiffness_scales"]):
            if i < articulation.num_joints:
                current_stiffness = articulation.get_joint_stiffness(i)
                articulation.set_joint_stiffness(i, current_stiffness * scale)
                print(f"  Joint {i} stiffness scale: {scale:.4f}")
    
    # Apply friction scales
    if "friction_scales" in params:
        for i, scale in enumerate(params["friction_scales"]):
            if i < articulation.num_joints:
                current_friction = articulation.get_joint_friction(i)
                articulation.set_joint_friction(i, current_friction * scale)
                print(f"  Joint {i} friction scale: {scale:.4f}")

def calculate_calibration_error(real_data, sim_data):
    """Calculate error between real and simulation data.
    
    Args:
        real_data (dict): Data from the physical robot
        sim_data (dict): Data from the simulation
    
    Returns:
        float: Error metric
    """
    # Calculate position error
    position_error = 0
    count = 0
    
    for real_pos, sim_pos in zip(real_data["joint_positions"], sim_data["joint_positions"]):
        # Convert to numpy arrays
        real_pos_np = np.array(real_pos)
        sim_pos_np = np.array(sim_pos)
        
        # Calculate mean absolute error
        if real_pos_np.shape == sim_pos_np.shape:
            position_error += np.mean(np.abs(real_pos_np - sim_pos_np))
            count += 1
    
    if count > 0:
        position_error /= count
    
    return position_error

def optimize_simulation_parameters(articulation, real_data):
    """Optimize simulation parameters to match real data.
    
    Args:
        articulation (ArticulationView): Isaac Sim articulation object
        real_data (dict): Data from the physical robot
    
    Returns:
        dict: Optimized parameters
    """
    print("Optimizing simulation parameters...")
    
    # Define parameters to optimize
    num_links = articulation.num_links
    num_joints = articulation.num_joints
    
    initial_params = {
        "mass_scales": [1.0] * num_links,
        "damping_scales": [1.0] * num_joints,
        "stiffness_scales": [1.0] * num_joints,
        "friction_scales": [1.0] * num_joints
    }
    
    # Flatten parameters for optimization
    initial_params_flat = []
    for param_list in initial_params.values():
        initial_params_flat.extend(param_list)
    
    initial_params_flat = np.array(initial_params_flat)
    
    # Get controller
    controller = articulation.get_articulation_controller()
    
    # Define objective function
    def objective(params_flat):
        # Reconstruct parameter dictionary
        idx = 0
        params = {}
        for key, value in initial_params.items():
            params[key] = params_flat[idx:idx+len(value)]
            idx += len(value)
        
        # Collect simulation data with these parameters
        sim_data = collect_sim_data(
            articulation,
            controller,
            real_data["trajectories"],
            params
        )
        
        # Calculate error
        error = calculate_calibration_error(real_data, sim_data)
        
        print(f"Parameters: {params_flat[:5]}... Error: {error:.6f}")
        
        return error
    
    # Run optimization
    print("Starting optimization (this may take a while)...")
    
    result = minimize(
        objective,
        initial_params_flat,
        method="Nelder-Mead",
        options={"maxiter": 20, "disp": True}
    )
    
    # Reconstruct optimized parameters
    idx = 0
    optimized_params = {}
    for key, value in initial_params.items():
        optimized_params[key] = result.x[idx:idx+len(value)].tolist()
        idx += len(value)
    
    print(f"Optimization completed with final error: {result.fun:.6f}")
    
    return optimized_params

def visualize_results(real_data, sim_data_before, sim_data_after):
    """Visualize calibration results.
    
    Args:
        real_data (dict): Data from the physical robot
        sim_data_before (dict): Simulation data before calibration
        sim_data_after (dict): Simulation data after calibration
    """
    print("Visualizing results...")
    
    # Create directory for plots if it doesn't exist
    os.makedirs(DATA_DIR, exist_ok=True)
    
    # Get number of joints
    num_joints = len(real_data["joint_positions"][0])
    
    # Create a figure for each joint
    for joint_idx in range(num_joints):
        plt.figure(figsize=(10, 6))
        
        # Extract data for this joint
        real_positions = [pos[joint_idx] for pos in real_data["joint_positions"]]
        sim_before_positions = [pos[joint_idx] for pos in sim_data_before["joint_positions"]]
        sim_after_positions = [pos[joint_idx] for pos in sim_data_after["joint_positions"]]
        
        # Plot data
        plt.plot(real_positions, 'b-', label='Real Robot')
        plt.plot(sim_before_positions, 'r--', label='Sim Before Calibration')
        plt.plot(sim_after_positions, 'g-', label='Sim After Calibration')
        
        plt.title(f'Joint {joint_idx+1} Position Comparison')
        plt.xlabel('Time Step')
        plt.ylabel('Joint Position (rad)')
        plt.legend()
        plt.grid(True)
        
        # Save figure
        plt.savefig(os.path.join(DATA_DIR, f'joint_{joint_idx+1}_comparison.png'))
        plt.close()
    
    # Create error comparison plot
    plt.figure(figsize=(10, 6))
    
    # Calculate errors
    errors_before = []
    errors_after = []
    
    for i in range(len(real_data["joint_positions"])):
        real_pos = np.array(real_data["joint_positions"][i])
        sim_before_pos = np.array(sim_data_before["joint_positions"][i])
        sim_after_pos = np.array(sim_data_after["joint_positions"][i])
        
        error_before = np.mean(np.abs(real_pos - sim_before_pos))
        error_after = np.mean(np.abs(real_pos - sim_after_pos))
        
        errors_before.append(error_before)
        errors_after.append(error_after)
    
    # Plot errors
    plt.plot(errors_before, 'r-', label='Error Before Calibration')
    plt.plot(errors_after, 'g-', label='Error After Calibration')
    
    plt.title('Calibration Error Comparison')
    plt.xlabel('Time Step')
    plt.ylabel('Mean Absolute Error (rad)')
    plt.legend()
    plt.grid(True)
    
    # Save figure
    plt.savefig(os.path.join(DATA_DIR, 'error_comparison.png'))
    plt.close()
    
    print(f"Visualization saved to {DATA_DIR}")

def save_parameters(params, filename):
    """Save parameters to a JSON file.
    
    Args:
        params (dict): Parameters to save
        filename (str): Output filename
    """
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    # Save parameters
    with open(filename, 'w') as f:
        json.dump(params, f, indent=4)
    
    print(f"Parameters saved to {filename}")

def load_parameters(filename):
    """Load parameters from a JSON file.
    
    Args:
        filename (str): Input filename
    
    Returns:
        dict: Loaded parameters
    """
    if not os.path.exists(filename):
        print(f"Parameter file {filename} not found")
        return None
    
    # Load parameters
    with open(filename, 'r') as f:
        params = json.load(f)
    
    print(f"Parameters loaded from {filename}")
    
    return params

def main():
    """Main function."""
    print("RoArm-M3 Pro Sim-to-Real Calibration Example")
    
    # Set up the scene
    setup_scene()
    
    # Create simulation context
    sim_context = SimulationContext()
    
    # Configure physics
    sim_context.set_physics_dt(SIMULATION_TIMESTEP)
    
    # Play the simulation
    sim_context.play()
    
    # Wait for physics to initialize
    time.sleep(1.0)
    
    # Get articulation
    arm_articulation = ArticulationView(
        prim_path="/World/RoArm_M3",
        name="roarm_articulation"
    )
    
    # Get articulation controller
    controller = arm_articulation.get_articulation_controller()
    
    # Get number of joints
    num_joints = arm_articulation.num_joints
    print(f"RoArm-M3 Pro has {num_joints} joints")
    
    # Create bridge to physical arm
    bridge = RoArmBridge(ARM_IP)
    
    # Generate test trajectories
    trajectories = generate_test_trajectories(num_joints)
    
    # Check if we should collect new data or use existing data
    param_file = os.path.join(DATA_DIR, "calibration_params.json")
    real_data_file = os.path.join(DATA_DIR, "real_data.json")
    
    if os.path.exists(real_data_file):
        print(f"Loading real data from {real_data_file}")
        with open(real_data_file, 'r') as f:
            real_data = json.load(f)
    else:
        # Collect data from the physical robot
        real_data = collect_real_data(bridge, trajectories)
        
        if real_data:
            # Save real data
            os.makedirs(os.path.dirname(real_data_file), exist_ok=True)
            with open(real_data_file, 'w') as f:
                json.dump(real_data, f)
    
    if not real_data:
        print("No real data available. Exiting.")
        sim_context.stop()
        return
    
    # Collect data from the simulation before calibration
    sim_data_before = collect_sim_data(arm_articulation, controller, real_data["trajectories"])
    
    # Check if we should load existing parameters
    if os.path.exists(param_file):
        print(f"Loading calibration parameters from {param_file}")
        optimized_params = load_parameters(param_file)
    else:
        # Optimize simulation parameters
        optimized_params = optimize_simulation_parameters(arm_articulation, real_data)
        
        # Save optimized parameters
        save_parameters(optimized_params, param_file)
    
    # Apply optimized parameters
    apply_parameters_to_sim(arm_articulation, optimized_params)
    
    # Collect data from the simulation after calibration
    sim_data_after = collect_sim_data(arm_articulation, controller, real_data["trajectories"])
    
    # Visualize results
    visualize_results(real_data, sim_data_before, sim_data_after)
    
    # Stop the simulation
    sim_context.stop()
    
    print("Calibration example completed")

if __name__ == "__main__":
    main()
