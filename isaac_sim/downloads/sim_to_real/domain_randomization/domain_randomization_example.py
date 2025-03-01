#!/usr/bin/env python3
# Domain Randomization Example for RoArm-M3 Pro with Isaac Sim
# This script demonstrates how to implement domain randomization techniques
# to improve sim-to-real transfer learning for the RoArm-M3 Pro robotic arm.

import numpy as np
import random
import os
import carb
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.world import World

class DomainRandomization:
    """Domain randomization for RoArm-M3 Pro in Isaac Sim"""
    
    def __init__(self, robot_usd_path, num_envs=1):
        """Initialize domain randomization for RoArm-M3 Pro
        
        Args:
            robot_usd_path (str): Path to the RoArm-M3 Pro USD file
            num_envs (int): Number of parallel environments
        """
        self.robot_usd_path = robot_usd_path
        self.num_envs = num_envs
        self.world = World(stage_units_in_meters=1.0)
        self.robots = None
        
    def load_robot(self):
        """Load the RoArm-M3 Pro robot into the scene"""
        self.world.scene.add_default_ground_plane()
        
        # Load robot
        robot_prim_path = "/World/RoArm_M3_Pro"
        add_reference_to_stage(self.robot_usd_path, robot_prim_path)
        
        # Setup robot view
        self.world.reset()
        self.robots = ArticulationView(prim_paths_expr=robot_prim_path, name="robot_view")
        self.world.scene.add(self.robots)
        
    def randomize_dynamics(self):
        """Randomize physical properties of the robot"""
        if self.robots is None:
            print("Error: Robot not loaded. Call load_robot() first.")
            return
            
        # Randomize joint properties
        for robot_idx in range(self.num_envs):
            for joint_idx in range(self.robots.num_joints):
                # Randomize damping (±20%)
                damping = self.robots.get_joint_damping(joint_idx)
                new_damping = damping * np.random.uniform(0.8, 1.2)
                self.robots.set_joint_damping(joint_idx, new_damping)
                
                # Randomize stiffness (±20%)
                stiffness = self.robots.get_joint_stiffness(joint_idx)
                new_stiffness = stiffness * np.random.uniform(0.8, 1.2)
                self.robots.set_joint_stiffness(joint_idx, new_stiffness)
                
                # Randomize friction (±20%)
                friction = self.robots.get_joint_friction(joint_idx)
                new_friction = friction * np.random.uniform(0.8, 1.2)
                self.robots.set_joint_friction(joint_idx, new_friction)
    
    def randomize_observation_noise(self, joint_positions):
        """Add noise to joint position observations
        
        Args:
            joint_positions (np.ndarray): Joint positions
            
        Returns:
            np.ndarray: Noisy joint positions
        """
        # Add Gaussian noise to joint positions (±0.5 degrees)
        noise = np.random.normal(0, np.deg2rad(0.5), joint_positions.shape)
        return joint_positions + noise
    
    def randomize_visual_appearance(self):
        """Randomize visual appearance of the environment"""
        # Randomize lighting
        light_prim_path = "/World/Light"
        if prim_utils.is_prim_path_valid(light_prim_path):
            intensity = random.uniform(500, 1500)
            prim_utils.set_prim_attribute(light_prim_path, "intensity", intensity)
            
            # Randomize light color
            r = random.uniform(0.9, 1.0)
            g = random.uniform(0.9, 1.0)
            b = random.uniform(0.9, 1.0)
            prim_utils.set_prim_attribute(light_prim_path, "color", (r, g, b))
    
    def randomize_mass_properties(self):
        """Randomize mass properties of the robot links"""
        if self.robots is None:
            print("Error: Robot not loaded. Call load_robot() first.")
            return
            
        # Get all links in the robot
        for robot_idx in range(self.num_envs):
            for link_idx in range(self.robots.num_links):
                # Randomize mass (±10%)
                mass = self.robots.get_link_mass(link_idx)
                new_mass = mass * np.random.uniform(0.9, 1.1)
                self.robots.set_link_mass(link_idx, new_mass)
    
    def randomize_all(self):
        """Apply all randomization techniques"""
        self.randomize_dynamics()
        self.randomize_visual_appearance()
        self.randomize_mass_properties()
        
    def run_example(self):
        """Run a simple example of domain randomization"""
        # Initialize simulation
        self.load_robot()
        
        # Run simulation with randomization
        for i in range(10):
            print(f"Iteration {i+1}/10: Applying domain randomization...")
            self.randomize_all()
            
            # Reset robot to a random pose
            random_positions = np.random.uniform(-0.5, 0.5, self.robots.num_joints)
            self.robots.set_joint_positions(random_positions)
            
            # Step simulation
            for _ in range(100):
                self.world.step(render=True)
                
                # Get joint positions with noise
                joint_positions = self.robots.get_joint_positions()
                noisy_positions = self.randomize_observation_noise(joint_positions)
                
                # Use noisy positions for control (in a real application)
                # ...
        
        print("Domain randomization example completed.")

def main():
    """Main function"""
    # Path to RoArm-M3 Pro USD file (replace with actual path)
    robot_usd_path = "/path/to/roarm_m3_pro.usd"
    
    # Create domain randomization instance
    domain_rand = DomainRandomization(robot_usd_path)
    
    # Run example
    domain_rand.run_example()

if __name__ == "__main__":
    main()
