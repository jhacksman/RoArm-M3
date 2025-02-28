#!/usr/bin/env python3
# System Identification Example for RoArm-M3 Pro with Isaac Sim
# This script demonstrates how to identify system parameters of the
# physical RoArm-M3 Pro robotic arm for accurate simulation.

import numpy as np
import matplotlib.pyplot as plt
import json
import os
import time
from scipy.optimize import minimize
from scipy.signal import savgol_filter

class SystemIdentification:
    """System identification for RoArm-M3 Pro"""
    
    def __init__(self, real_robot_ip="192.168.4.1"):
        """Initialize system identification for RoArm-M3 Pro
        
        Args:
            real_robot_ip (str): IP address of the physical RoArm-M3 Pro
        """
        self.real_robot_ip = real_robot_ip
        self.collected_data = None
        self.identified_params = None
        
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
        
    def generate_identification_trajectory(self):
        """Generate a trajectory for system identification
        
        Returns:
            np.ndarray: Joint positions trajectory
        """
        # Generate a trajectory that excites the system dynamics
        num_points = 200
        num_joints = 6  # RoArm-M3 Pro has 6 joints
        
        # Create a trajectory with multiple frequency components
        trajectory = np.zeros((num_points, num_joints))
        for joint in range(num_joints):
            # Use multiple sine waves with different frequencies
            for freq_idx in range(3):
                amplitude = np.radians(15) / (freq_idx + 1)  # Decreasing amplitude
                frequency = 0.2 + joint * 0.1 + freq_idx * 0.3  # Different frequencies
                phase = joint * np.pi / 3 + freq_idx * np.pi / 6  # Different phases
                
                for i in range(num_points):
                    t = i / num_points * 20  # 20 seconds trajectory
                    trajectory[i, joint] += amplitude * np.sin(frequency * t + phase)
                
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
        # and record the resulting joint positions, velocities, torques, and accelerations
        # For this example, we'll simulate the data collection
        
        num_points = trajectory.shape[0]
        num_joints = trajectory.shape[1]
        
        # Simulate real robot data with realistic dynamics
        positions = np.zeros((num_points, num_joints))
        velocities = np.zeros((num_points, num_joints))
        accelerations = np.zeros((num_points, num_joints))
        torques = np.zeros((num_points, num_joints))
        
        # Define some realistic parameters for the simulation
        mass = np.array([0.2, 0.15, 0.12, 0.1, 0.08, 0.05])  # kg
        damping = np.array([0.1, 0.08, 0.08, 0.06, 0.06, 0.04])
        friction = np.array([0.05, 0.04, 0.04, 0.03, 0.03, 0.02])
        inertia = np.array([0.01, 0.008, 0.006, 0.004, 0.003, 0.002])
        
        # Simulate the robot dynamics
        for i in range(num_points):
            # Add tracking error and delay
            if i > 0:
                # Calculate acceleration (second derivative of position)
                if i > 1:
                    accelerations[i-1] = (positions[i] - 2*positions[i-1] + positions[i-2]) / (0.1**2)
                
                # Simulate dynamics with mass, damping, friction, and inertia
                for j in range(num_joints):
                    # Calculate torque based on the dynamics model
                    if i > 1:
                        torques[i-1, j] = (
                            mass[j] * accelerations[i-1, j] +  # Inertial term
                            damping[j] * velocities[i-1, j] +  # Damping term
                            friction[j] * np.sign(velocities[i-1, j]) +  # Friction term
                            0.5 * np.sin(positions[i-1, j])  # Gravity term
                        )
                
                # Simulate tracking delay and error
                delay_factor = 0.9
                noise_factor = 0.01
                positions[i] = delay_factor * trajectory[i] + (1 - delay_factor) * positions[i-1]
                positions[i] += np.random.normal(0, noise_factor, num_joints)
                
                # Calculate velocities (with noise)
                velocities[i] = (positions[i] - positions[i-1]) / 0.1  # Assuming 10Hz
                velocities[i] += np.random.normal(0, 0.005, num_joints)
            else:
                positions[i] = trajectory[i]
        
        # Apply a filter to smooth the data (as would be done with real data)
        for j in range(num_joints):
            positions[:, j] = savgol_filter(positions[:, j], 15, 3)
            velocities[:, j] = savgol_filter(velocities[:, j], 15, 3)
            torques[:, j] = savgol_filter(torques[:, j], 15, 3)
        
        # Store the collected data
        self.collected_data = {
            "timestamp": np.arange(num_points) * 0.1,  # 10Hz
            "commanded_positions": trajectory,
            "actual_positions": positions,
            "velocities": velocities,
            "accelerations": accelerations,
            "torques": torques
        }
        
        return self.collected_data
        
    def identify_parameters(self):
        """Identify system parameters from collected data
        
        Returns:
            dict: Identified parameters
        """
        if self.collected_data is None:
            print("No data available. Run collect_real_robot_data() first.")
            return None
            
        print("Identifying system parameters...")
        
        # Extract data
        positions = self.collected_data["actual_positions"]
        velocities = self.collected_data["velocities"]
        accelerations = self.collected_data["accelerations"]
        torques = self.collected_data["torques"]
        
        num_joints = positions.shape[1]
        
        # Initialize parameters
        identified_params = {
            "mass": np.zeros(num_joints),
            "damping": np.zeros(num_joints),
            "friction": np.zeros(num_joints),
            "gravity_coef": np.zeros(num_joints)
        }
        
        # Identify parameters for each joint
        for joint in range(num_joints):
            print(f"Identifying parameters for joint {joint+1}...")
            
            # Prepare data for this joint
            pos = positions[:, joint]
            vel = velocities[:, joint]
            acc = accelerations[:, joint]
            torque = torques[:, joint]
            
            # Create feature matrix X
            X = np.column_stack([
                acc,  # Mass term
                vel,  # Damping term
                np.sign(vel),  # Friction term
                np.sin(pos)  # Gravity term
            ])
            
            # Solve least squares problem: X * params = torque
            params, residuals, _, _ = np.linalg.lstsq(X, torque, rcond=None)
            
            # Store identified parameters
            identified_params["mass"][joint] = params[0]
            identified_params["damping"][joint] = params[1]
            identified_params["friction"][joint] = params[2]
            identified_params["gravity_coef"][joint] = params[3]
            
            print(f"  Mass: {params[0]:.4f} kg")
            print(f"  Damping: {params[1]:.4f} Nms/rad")
            print(f"  Friction: {params[2]:.4f} Nm")
            print(f"  Gravity coefficient: {params[3]:.4f} Nm")
            
            if residuals.size > 0:
                print(f"  Residual error: {residuals[0]:.4f}")
            
        self.identified_params = identified_params
        return identified_params
        
    def validate_parameters(self, test_trajectory=None):
        """Validate identified parameters with a test trajectory
        
        Args:
            test_trajectory (np.ndarray, optional): Test trajectory
            
        Returns:
            float: Validation error
        """
        if self.identified_params is None:
            print("No identified parameters. Run identify_parameters() first.")
            return None
            
        print("Validating identified parameters...")
        
        # Generate a test trajectory if not provided
        if test_trajectory is None:
            num_points = 100
            num_joints = len(self.identified_params["mass"])
            test_trajectory = np.zeros((num_points, num_joints))
            
            for joint in range(num_joints):
                amplitude = np.radians(20)
                frequency = 0.3 + joint * 0.05
                phase = joint * np.pi / 4
                
                for i in range(num_points):
                    t = i / num_points * 10  # 10 seconds trajectory
                    test_trajectory[i, joint] = amplitude * np.sin(frequency * t + phase)
        
        # Collect real data for the test trajectory
        real_data = self.collect_real_robot_data(test_trajectory)
        
        # Simulate the robot with identified parameters
        num_points = test_trajectory.shape[0]
        num_joints = test_trajectory.shape[1]
        
        sim_positions = np.zeros((num_points, num_joints))
        sim_velocities = np.zeros((num_points, num_joints))
        sim_accelerations = np.zeros((num_points, num_joints))
        sim_torques = np.zeros((num_points, num_joints))
        
        # Extract identified parameters
        mass = self.identified_params["mass"]
        damping = self.identified_params["damping"]
        friction = self.identified_params["friction"]
        gravity_coef = self.identified_params["gravity_coef"]
        
        # Simulate the robot dynamics with identified parameters
        for i in range(num_points):
            if i > 0:
                # Calculate acceleration (second derivative of position)
                if i > 1:
                    sim_accelerations[i-1] = (sim_positions[i] - 2*sim_positions[i-1] + sim_positions[i-2]) / (0.1**2)
                
                # Calculate torque based on the dynamics model
                for j in range(num_joints):
                    if i > 1:
                        sim_torques[i-1, j] = (
                            mass[j] * sim_accelerations[i-1, j] +  # Inertial term
                            damping[j] * sim_velocities[i-1, j] +  # Damping term
                            friction[j] * np.sign(sim_velocities[i-1, j]) +  # Friction term
                            gravity_coef[j] * np.sin(sim_positions[i-1, j])  # Gravity term
                        )
                
                # Simulate tracking delay and error
                delay_factor = 0.9
                sim_positions[i] = delay_factor * test_trajectory[i] + (1 - delay_factor) * sim_positions[i-1]
                
                # Calculate velocities
                sim_velocities[i] = (sim_positions[i] - sim_positions[i-1]) / 0.1  # Assuming 10Hz
            else:
                sim_positions[i] = test_trajectory[i]
        
        # Calculate validation error
        pos_error = np.mean((sim_positions - real_data["actual_positions"])**2)
        vel_error = np.mean((sim_velocities - real_data["velocities"])**2)
        torque_error = np.mean((sim_torques - real_data["torques"])**2)
        
        total_error = pos_error + 0.5 * vel_error + 0.3 * torque_error
        
        print(f"Validation error:")
        print(f"  Position RMSE: {np.sqrt(pos_error):.4f} rad")
        print(f"  Velocity RMSE: {np.sqrt(vel_error):.4f} rad/s")
        print(f"  Torque RMSE: {np.sqrt(torque_error):.4f} Nm")
        print(f"  Total weighted error: {total_error:.4f}")
        
        # Visualize the results
        plt.figure(figsize=(12, 8))
        for joint in range(min(num_joints, 6)):  # Show up to 6 joints
            plt.subplot(3, 2, joint+1)
            plt.plot(real_data["timestamp"], real_data["actual_positions"][:, joint], 'b-', label='Real Robot')
            plt.plot(real_data["timestamp"], sim_positions[:, joint], 'r--', label='Simulation')
            plt.title(f'Joint {joint+1} Position')
            plt.xlabel('Time (s)')
            plt.ylabel('Position (rad)')
            plt.legend()
            
        plt.tight_layout()
        plt.savefig('validation_results.png')
        print("Validation results saved to 'validation_results.png'")
        
        return total_error
        
    def export_parameters_to_isaac_sim(self, output_file='isaac_sim_params.json'):
        """Export identified parameters to a format usable by Isaac Sim
        
        Args:
            output_file (str): Output filename
            
        Returns:
            bool: True if successful, False otherwise
        """
        if self.identified_params is None:
            print("No identified parameters. Run identify_parameters() first.")
            return False
            
        print(f"Exporting parameters to Isaac Sim format: {output_file}")
        
        # Convert parameters to Isaac Sim format
        isaac_params = {
            "articulation_params": {
                "joints": []
            }
        }
        
        num_joints = len(self.identified_params["mass"])
        for joint in range(num_joints):
            joint_params = {
                "joint_index": joint,
                "damping": float(self.identified_params["damping"][joint]),
                "stiffness": 0.0,  # Default value
                "friction": float(self.identified_params["friction"][joint]),
                "armature": float(self.identified_params["mass"][joint] * 0.01)  # Convert to armature
            }
            isaac_params["articulation_params"]["joints"].append(joint_params)
            
        # Add link mass properties
        isaac_params["link_params"] = {
            "masses": self.identified_params["mass"].tolist()
        }
        
        # Save to file
        with open(output_file, 'w') as f:
            json.dump(isaac_params, f, indent=4)
            
        print(f"Parameters exported to '{output_file}'")
        return True
        
    def run_identification(self):
        """Run the complete system identification process
        
        Returns:
            dict: Identified parameters
        """
        # Connect to the physical robot
        if not self.connect_to_real_robot():
            print("Failed to connect to the physical robot.")
            return None
            
        # Generate identification trajectory
        trajectory = self.generate_identification_trajectory()
        
        # Collect data from the physical robot
        self.collect_real_robot_data(trajectory)
        
        # Identify parameters
        self.identify_parameters()
        
        # Validate parameters
        self.validate_parameters()
        
        # Export parameters to Isaac Sim format
        self.export_parameters_to_isaac_sim()
        
        return self.identified_params

def main():
    """Main function"""
    # Create system identification instance
    sys_id = SystemIdentification()
    
    # Run identification
    sys_id.run_identification()

if __name__ == "__main__":
    main()
