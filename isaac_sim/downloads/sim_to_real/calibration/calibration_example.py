#!/usr/bin/env python3
# Calibration Example for RoArm-M3 Pro with Isaac Sim
# This script demonstrates how to calibrate simulation parameters
# to match the physical RoArm-M3 Pro robotic arm.

import numpy as np
import matplotlib.pyplot as plt
import os
import json
import time
from scipy.optimize import minimize

class SimulationCalibration:
    """Calibration for RoArm-M3 Pro in Isaac Sim"""
    
    def __init__(self, robot_usd_path, real_robot_ip="192.168.4.1"):
        """Initialize calibration for RoArm-M3 Pro
        
        Args:
            robot_usd_path (str): Path to the RoArm-M3 Pro USD file
            real_robot_ip (str): IP address of the physical RoArm-M3 Pro
        """
        self.robot_usd_path = robot_usd_path
        self.real_robot_ip = real_robot_ip
        self.sim_data = None
        self.real_data = None
        self.calibrated_params = None
        
    def connect_to_real_robot(self):
        """Connect to the physical RoArm-M3 Pro robot
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        print(f"Connecting to RoArm-M3 Pro at {self.real_robot_ip}...")
        # In a real implementation, this would establish a connection
        # to the physical robot using HTTP, Serial, or ROS2
        # For this example, we'll simulate a successful connection
        return True
        
    def generate_test_trajectory(self):
        """Generate a test trajectory for calibration
        
        Returns:
            np.ndarray: Joint positions trajectory
        """
        # Generate a smooth trajectory that covers the robot's workspace
        num_points = 100
        num_joints = 6  # RoArm-M3 Pro has 6 joints
        
        # Create a sinusoidal trajectory for each joint
        trajectory = np.zeros((num_points, num_joints))
        for joint in range(num_joints):
            amplitude = np.radians(30)  # 30 degrees
            frequency = 0.5 + joint * 0.1  # Different frequency for each joint
            phase = joint * np.pi / 3  # Different phase for each joint
            
            for i in range(num_points):
                t = i / num_points * 10  # 10 seconds trajectory
                trajectory[i, joint] = amplitude * np.sin(frequency * t + phase)
                
        return trajectory
        
    def collect_real_robot_data(self, trajectory):
        """Collect data from the physical robot
        
        Args:
            trajectory (np.ndarray): Joint positions trajectory
            
        Returns:
            dict: Collected data from the physical robot
        """
        print("Collecting data from the physical RoArm-M3 Pro...")
        
        # In a real implementation, this would send commands to the physical robot
        # and record the resulting joint positions, velocities, and torques
        # For this example, we'll simulate the data collection
        
        num_points = trajectory.shape[0]
        num_joints = trajectory.shape[1]
        
        # Simulate real robot data with some noise and dynamics effects
        positions = np.zeros((num_points, num_joints))
        velocities = np.zeros((num_points, num_joints))
        torques = np.zeros((num_points, num_joints))
        
        for i in range(num_points):
            # Add tracking error and delay
            if i > 0:
                # Simulate tracking delay and error
                delay_factor = 0.9
                noise_factor = 0.02
                positions[i] = delay_factor * trajectory[i] + (1 - delay_factor) * positions[i-1]
                positions[i] += np.random.normal(0, noise_factor, num_joints)
                
                # Calculate velocities (with noise)
                velocities[i] = (positions[i] - positions[i-1]) / 0.1  # Assuming 10Hz
                velocities[i] += np.random.normal(0, 0.01, num_joints)
                
                # Simulate torques based on simple dynamics model
                torques[i] = 0.5 * positions[i] + 0.3 * velocities[i]
                torques[i] += np.random.normal(0, 0.05, num_joints)
            else:
                positions[i] = trajectory[i]
        
        return {
            "timestamp": np.arange(num_points) * 0.1,  # 10Hz
            "commanded_positions": trajectory,
            "actual_positions": positions,
            "velocities": velocities,
            "torques": torques
        }
        
    def collect_simulation_data(self, trajectory, params):
        """Collect data from the simulation with given parameters
        
        Args:
            trajectory (np.ndarray): Joint positions trajectory
            params (dict): Simulation parameters
            
        Returns:
            dict: Collected data from the simulation
        """
        print("Collecting data from Isaac Sim simulation...")
        
        # In a real implementation, this would set up the simulation with the given parameters,
        # send commands to the simulated robot, and record the resulting data
        # For this example, we'll simulate the data collection
        
        num_points = trajectory.shape[0]
        num_joints = trajectory.shape[1]
        
        # Extract parameters
        damping = params.get("damping", 1.0)
        stiffness = params.get("stiffness", 1.0)
        friction = params.get("friction", 1.0)
        mass_scale = params.get("mass_scale", 1.0)
        
        # Simulate robot data based on parameters
        positions = np.zeros((num_points, num_joints))
        velocities = np.zeros((num_points, num_joints))
        torques = np.zeros((num_points, num_joints))
        
        for i in range(num_points):
            # Simulate dynamics based on parameters
            if i > 0:
                # Higher damping means slower response
                delay_factor = 0.9 - 0.1 * (damping - 1.0)
                delay_factor = max(0.7, min(0.95, delay_factor))
                
                # Higher friction means more tracking error
                noise_factor = 0.01 + 0.01 * (friction - 1.0)
                
                positions[i] = delay_factor * trajectory[i] + (1 - delay_factor) * positions[i-1]
                positions[i] += np.random.normal(0, noise_factor, num_joints)
                
                # Calculate velocities
                velocities[i] = (positions[i] - positions[i-1]) / 0.1  # Assuming 10Hz
                
                # Calculate torques based on parameters
                torques[i] = (0.3 * mass_scale) * positions[i] + (0.2 * damping) * velocities[i] + (0.1 * friction)
            else:
                positions[i] = trajectory[i]
        
        return {
            "timestamp": np.arange(num_points) * 0.1,  # 10Hz
            "commanded_positions": trajectory,
            "actual_positions": positions,
            "velocities": velocities,
            "torques": torques,
            "parameters": params
        }
        
    def calculate_error(self, sim_data, real_data):
        """Calculate error between simulation and real data
        
        Args:
            sim_data (dict): Simulation data
            real_data (dict): Real robot data
            
        Returns:
            float: Error metric
        """
        # Calculate RMSE for positions, velocities, and torques
        pos_error = np.mean((sim_data["actual_positions"] - real_data["actual_positions"])**2)
        vel_error = np.mean((sim_data["velocities"] - real_data["velocities"])**2)
        torque_error = np.mean((sim_data["torques"] - real_data["torques"])**2)
        
        # Weighted sum of errors
        total_error = pos_error + 0.5 * vel_error + 0.3 * torque_error
        return total_error
        
    def objective_function(self, param_vector):
        """Objective function for optimization
        
        Args:
            param_vector (np.ndarray): Parameter vector [damping, stiffness, friction, mass_scale]
            
        Returns:
            float: Error metric
        """
        # Convert parameter vector to dictionary
        params = {
            "damping": param_vector[0],
            "stiffness": param_vector[1],
            "friction": param_vector[2],
            "mass_scale": param_vector[3]
        }
        
        # Collect simulation data with these parameters
        sim_data = self.collect_simulation_data(self.real_data["commanded_positions"], params)
        
        # Calculate error
        error = self.calculate_error(sim_data, self.real_data)
        print(f"Parameters: {params}, Error: {error}")
        return error
        
    def calibrate(self):
        """Calibrate simulation parameters to match the physical robot
        
        Returns:
            dict: Calibrated parameters
        """
        print("Starting calibration process...")
        
        # Connect to the physical robot
        if not self.connect_to_real_robot():
            print("Failed to connect to the physical robot.")
            return None
        
        # Generate test trajectory
        trajectory = self.generate_test_trajectory()
        
        # Collect data from the physical robot
        self.real_data = self.collect_real_robot_data(trajectory)
        
        # Initial parameters
        initial_params = np.array([1.0, 1.0, 1.0, 1.0])  # [damping, stiffness, friction, mass_scale]
        
        # Parameter bounds
        bounds = [(0.5, 2.0), (0.5, 2.0), (0.5, 2.0), (0.5, 2.0)]
        
        # Optimize parameters
        print("Optimizing simulation parameters...")
        result = minimize(
            self.objective_function,
            initial_params,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 20}
        )
        
        # Convert optimized parameters to dictionary
        self.calibrated_params = {
            "damping": result.x[0],
            "stiffness": result.x[1],
            "friction": result.x[2],
            "mass_scale": result.x[3]
        }
        
        print(f"Calibration completed. Optimized parameters: {self.calibrated_params}")
        return self.calibrated_params
        
    def apply_calibrated_parameters(self):
        """Apply calibrated parameters to the simulation
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.calibrated_params is None:
            print("No calibrated parameters available. Run calibrate() first.")
            return False
            
        print(f"Applying calibrated parameters to simulation: {self.calibrated_params}")
        
        # In a real implementation, this would update the simulation parameters
        # For this example, we'll just return True
        return True
        
    def visualize_results(self):
        """Visualize calibration results
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.real_data is None or self.calibrated_params is None:
            print("No data available for visualization. Run calibrate() first.")
            return False
            
        # Collect simulation data with calibrated parameters
        sim_data = self.collect_simulation_data(self.real_data["commanded_positions"], self.calibrated_params)
        
        # Plot joint positions
        plt.figure(figsize=(12, 8))
        for joint in range(self.real_data["actual_positions"].shape[1]):
            plt.subplot(3, 2, joint+1)
            plt.plot(self.real_data["timestamp"], self.real_data["actual_positions"][:, joint], 'b-', label='Real Robot')
            plt.plot(sim_data["timestamp"], sim_data["actual_positions"][:, joint], 'r--', label='Simulation')
            plt.plot(self.real_data["timestamp"], self.real_data["commanded_positions"][:, joint], 'g:', label='Commanded')
            plt.title(f'Joint {joint+1} Position')
            plt.xlabel('Time (s)')
            plt.ylabel('Position (rad)')
            plt.legend()
            
        plt.tight_layout()
        plt.savefig('calibration_results.png')
        print("Visualization saved to 'calibration_results.png'")
        return True
        
    def save_calibrated_parameters(self, filename='calibrated_params.json'):
        """Save calibrated parameters to a file
        
        Args:
            filename (str): Output filename
            
        Returns:
            bool: True if successful, False otherwise
        """
        if self.calibrated_params is None:
            print("No calibrated parameters available. Run calibrate() first.")
            return False
            
        with open(filename, 'w') as f:
            json.dump(self.calibrated_params, f, indent=4)
            
        print(f"Calibrated parameters saved to '{filename}'")
        return True
        
    def load_calibrated_parameters(self, filename='calibrated_params.json'):
        """Load calibrated parameters from a file
        
        Args:
            filename (str): Input filename
            
        Returns:
            dict: Loaded parameters
        """
        if not os.path.exists(filename):
            print(f"File '{filename}' not found.")
            return None
            
        with open(filename, 'r') as f:
            self.calibrated_params = json.load(f)
            
        print(f"Calibrated parameters loaded from '{filename}': {self.calibrated_params}")
        return self.calibrated_params

def main():
    """Main function"""
    # Path to RoArm-M3 Pro USD file (replace with actual path)
    robot_usd_path = "/path/to/roarm_m3_pro.usd"
    
    # Create calibration instance
    calibration = SimulationCalibration(robot_usd_path)
    
    # Run calibration
    calibration.calibrate()
    
    # Apply calibrated parameters
    calibration.apply_calibrated_parameters()
    
    # Visualize results
    calibration.visualize_results()
    
    # Save calibrated parameters
    calibration.save_calibrated_parameters()

if __name__ == "__main__":
    main()
