#!/usr/bin/env python3
"""
Joint Control Example for RoArm-M3 Pro in NVIDIA Isaac Sim

This example demonstrates how to control individual joints of the
RoArm-M3 Pro robotic arm in NVIDIA Isaac Sim.
"""

import os
import time
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.articulations import ArticulationView

# Configuration
ARM_USD_PATH = "/path/to/roarm_m3.usd"  # Update with your USD path
SIMULATION_TIMESTEP = 1.0 / 60.0  # 60 Hz physics simulation

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
    else:
        print(f"USD model not found at {ARM_USD_PATH}")
        print("Creating simplified arm model instead")
        create_simplified_arm()

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
    
    # Create joints and links
    create_joint_and_link(
        parent_path="/World/RoArm_M3/base",
        link_path="/World/RoArm_M3/shoulder",
        joint_path="/World/RoArm_M3/base/shoulder_joint",
        link_size=(0.05, 0.05, 0.15),
        link_offset=(0, 0, 0.1),
        joint_axis=(0, 1, 0),
        joint_limits=(-1.57, 1.57)
    )
    
    create_joint_and_link(
        parent_path="/World/RoArm_M3/shoulder",
        link_path="/World/RoArm_M3/elbow",
        joint_path="/World/RoArm_M3/shoulder/elbow_joint",
        link_size=(0.05, 0.05, 0.15),
        link_offset=(0, 0, 0.15),
        joint_axis=(0, 1, 0),
        joint_limits=(-1.57, 1.57)
    )
    
    create_joint_and_link(
        parent_path="/World/RoArm_M3/elbow",
        link_path="/World/RoArm_M3/wrist",
        joint_path="/World/RoArm_M3/elbow/wrist_joint",
        link_size=(0.05, 0.05, 0.1),
        link_offset=(0, 0, 0.125),
        joint_axis=(0, 1, 0),
        joint_limits=(-1.57, 1.57)
    )
    
    # Create end effector
    end_effector_prim = prim_utils.create_prim(
        "/World/RoArm_M3/gripper",
        "Cube",
        attributes={"size": (0.05, 0.1, 0.05)}
    )
    prim_utils.set_attribute(end_effector_prim.GetPath(), "xformOp:translate", (0, 0, 0.35))
    
    # Create gripper joint
    gripper_joint_prim = prim_utils.create_prim(
        "/World/RoArm_M3/wrist/gripper_joint",
        "PhysicsJoint"
    )
    prim_utils.set_attribute(gripper_joint_prim.GetPath(), "type", "revolute")
    prim_utils.set_attribute(gripper_joint_prim.GetPath(), "body0", "/World/RoArm_M3/wrist")
    prim_utils.set_attribute(gripper_joint_prim.GetPath(), "body1", "/World/RoArm_M3/gripper")
    prim_utils.set_attribute(gripper_joint_prim.GetPath(), "axis", (0, 0, 1))
    prim_utils.set_attribute(gripper_joint_prim.GetPath(), "lowerLimit", -3.14)
    prim_utils.set_attribute(gripper_joint_prim.GetPath(), "upperLimit", 3.14)

def create_joint_and_link(parent_path, link_path, joint_path, link_size, link_offset, joint_axis, joint_limits):
    """Create a joint and link pair."""
    # Create link
    link_prim = prim_utils.create_prim(
        link_path,
        "Cube",
        attributes={"size": link_size}
    )
    prim_utils.set_attribute(link_prim.GetPath(), "xformOp:translate", link_offset)
    
    # Create joint
    joint_prim = prim_utils.create_prim(
        joint_path,
        "PhysicsJoint"
    )
    prim_utils.set_attribute(joint_prim.GetPath(), "type", "revolute")
    prim_utils.set_attribute(joint_prim.GetPath(), "body0", parent_path)
    prim_utils.set_attribute(joint_prim.GetPath(), "body1", link_path)
    prim_utils.set_attribute(joint_prim.GetPath(), "axis", joint_axis)
    prim_utils.set_attribute(joint_prim.GetPath(), "lowerLimit", joint_limits[0])
    prim_utils.set_attribute(joint_prim.GetPath(), "upperLimit", joint_limits[1])

def control_individual_joints():
    """Control individual joints of the RoArm-M3 Pro."""
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
    
    # Get joint names
    joint_names = arm_articulation.get_joint_names()
    print(f"Joint names: {joint_names}")
    
    # Get joint positions
    joint_positions = arm_articulation.get_joint_positions()
    print(f"Initial joint positions: {joint_positions}")
    
    # Get joint limits
    joint_limits = []
    for i in range(num_joints):
        lower_limit = arm_articulation.get_joint_limits()[0][i]
        upper_limit = arm_articulation.get_joint_limits()[1][i]
        joint_limits.append((lower_limit, upper_limit))
    
    print(f"Joint limits: {joint_limits}")
    
    # Control each joint individually
    for joint_idx in range(num_joints):
        print(f"\nControlling joint {joint_idx} ({joint_names[joint_idx] if joint_idx < len(joint_names) else 'unnamed'})")
        
        # Get joint limits
        if joint_idx < len(joint_limits):
            lower_limit, upper_limit = joint_limits[joint_idx]
        else:
            lower_limit, upper_limit = -1.57, 1.57  # Default limits
        
        # Move joint from lower to upper limit
        steps = 50
        for step in range(steps + 1):
            # Calculate target position
            alpha = step / steps
            target_position = lower_limit + alpha * (upper_limit - lower_limit)
            
            # Create target positions array
            target_positions = np.zeros(num_joints)
            target_positions[joint_idx] = target_position
            
            # Apply action
            controller.apply_action(target_positions)
            
            # Step simulation
            for _ in range(5):  # Multiple steps for smoother motion
                sim_context.step()
            
            # Print current position
            current_positions = arm_articulation.get_joint_positions()
            print(f"Step {step}: Joint {joint_idx} position = {current_positions[joint_idx]:.4f}")
        
        # Move joint back to zero
        steps = 20
        for step in range(steps + 1):
            # Calculate target position
            alpha = step / steps
            target_position = upper_limit * (1 - alpha)
            
            # Create target positions array
            target_positions = np.zeros(num_joints)
            target_positions[joint_idx] = target_position
            
            # Apply action
            controller.apply_action(target_positions)
            
            # Step simulation
            for _ in range(5):  # Multiple steps for smoother motion
                sim_context.step()
    
    # Move all joints in a synchronized pattern
    print("\nMoving all joints in a synchronized pattern")
    
    # Run for 300 steps (about 5 seconds at 60Hz)
    for i in range(300):
        # Calculate target positions with sinusoidal motion
        t = i * SIMULATION_TIMESTEP
        target_positions = np.zeros(num_joints)
        
        for j in range(num_joints):
            # Use different frequencies and phases for each joint
            frequency = 0.5 + j * 0.2
            phase = j * np.pi / 4
            
            # Calculate position within joint limits
            if j < len(joint_limits):
                lower_limit, upper_limit = joint_limits[j]
                mid_point = (lower_limit + upper_limit) / 2
                amplitude = (upper_limit - lower_limit) / 2 * 0.8  # 80% of range
                target_positions[j] = mid_point + amplitude * np.sin(frequency * t + phase)
            else:
                # Default motion if limits not available
                target_positions[j] = np.sin(frequency * t + phase)
        
        # Apply action
        controller.apply_action(target_positions)
        
        # Step simulation
        sim_context.step()
        
        # Print progress every 30 steps
        if i % 30 == 0:
            current_positions = arm_articulation.get_joint_positions()
            print(f"Step {i}: Joint positions = {current_positions}")
    
    # Stop the simulation
    sim_context.stop()
    
    print("Joint control example completed")

def main():
    """Main function."""
    print("RoArm-M3 Pro Joint Control Example")
    
    # Set up the scene
    setup_scene()
    
    # Control joints
    control_individual_joints()

if __name__ == "__main__":
    main()
