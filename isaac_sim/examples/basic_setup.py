#!/usr/bin/env python3
"""
Basic Setup Example for RoArm-M3 Pro in NVIDIA Isaac Sim

This example demonstrates how to set up a basic simulation environment
with the RoArm-M3 Pro robotic arm in NVIDIA Isaac Sim.
"""

import os
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.rotations as rot_utils
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.articulations import ArticulationView

# Configuration
ARM_USD_PATH = "/path/to/roarm_m3.usd"  # Update with your USD path
SIMULATION_TIMESTEP = 1.0 / 60.0  # 60 Hz physics simulation
RENDERING_TIMESTEP = 1.0 / 30.0  # 30 Hz rendering

def create_basic_environment():
    """Create a basic simulation environment."""
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
    
    # Add collision properties to ground
    prim_utils.set_collision_properties(
        ground_prim.GetPath(),
        collision_enabled=True,
        contact_offset=0.02,
        rest_offset=0.0
    )
    
    # Add lighting
    dome_light = prim_utils.create_prim(
        "/World/Lights/DomeLight",
        "DomeLight"
    )
    prim_utils.set_attribute(dome_light.GetPath(), "intensity", 1000.0)
    prim_utils.set_attribute(dome_light.GetPath(), "color", (1.0, 1.0, 1.0))
    
    # Add key light
    key_light = prim_utils.create_prim(
        "/World/Lights/KeyLight",
        "DistantLight"
    )
    prim_utils.set_attribute(key_light.GetPath(), "intensity", 2500.0)
    prim_utils.set_attribute(key_light.GetPath(), "color", (1.0, 0.98, 0.95))
    prim_utils.set_attribute(key_light.GetPath(), "rotation", rot_utils.euler_angles_to_quat([np.pi/4, 0, 0]))

def create_simplified_arm():
    """Create a simplified RoArm-M3 Pro model if USD file is not available."""
    # Create base
    base_prim = prim_utils.create_prim(
        "/World/RoArm_M3/base",
        "Cylinder",
        attributes={"radius": 0.1, "height": 0.05}
    )
    
    # Create articulation root
    prim_utils.set_articulation_root_properties(base_prim.GetPath())
    
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
    
    # Create second arm segment
    arm2_prim = prim_utils.create_prim(
        "/World/RoArm_M3/arm2",
        "Cube",
        attributes={"size": (0.05, 0.05, 0.2)}
    )
    prim_utils.set_attribute(arm2_prim.GetPath(), "xformOp:translate", (0, 0, 0.35))
    
    # Create second joint
    joint2_prim = prim_utils.create_prim(
        "/World/RoArm_M3/arm1/joint2",
        "PhysicsJoint"
    )
    prim_utils.set_attribute(joint2_prim.GetPath(), "type", "revolute")
    prim_utils.set_attribute(joint2_prim.GetPath(), "body0", "/World/RoArm_M3/arm1")
    prim_utils.set_attribute(joint2_prim.GetPath(), "body1", "/World/RoArm_M3/arm2")
    prim_utils.set_attribute(joint2_prim.GetPath(), "axis", (0, 1, 0))
    prim_utils.set_attribute(joint2_prim.GetPath(), "lowerLimit", -1.57)
    prim_utils.set_attribute(joint2_prim.GetPath(), "upperLimit", 1.57)
    
    # Create end effector
    end_effector_prim = prim_utils.create_prim(
        "/World/RoArm_M3/end_effector",
        "Cube",
        attributes={"size": (0.05, 0.1, 0.05)}
    )
    prim_utils.set_attribute(end_effector_prim.GetPath(), "xformOp:translate", (0, 0, 0.5))
    
    # Create end effector joint
    joint3_prim = prim_utils.create_prim(
        "/World/RoArm_M3/arm2/joint3",
        "PhysicsJoint"
    )
    prim_utils.set_attribute(joint3_prim.GetPath(), "type", "revolute")
    prim_utils.set_attribute(joint3_prim.GetPath(), "body0", "/World/RoArm_M3/arm2")
    prim_utils.set_attribute(joint3_prim.GetPath(), "body1", "/World/RoArm_M3/end_effector")
    prim_utils.set_attribute(joint3_prim.GetPath(), "axis", (0, 1, 0))
    prim_utils.set_attribute(joint3_prim.GetPath(), "lowerLimit", -1.57)
    prim_utils.set_attribute(joint3_prim.GetPath(), "upperLimit", 1.57)

def import_arm_model():
    """Import the RoArm-M3 Pro USD model."""
    if os.path.exists(ARM_USD_PATH):
        # Import USD model
        from omni.kit.commands import execute
        execute("CreateReferenceCommand",
            asset_path=ARM_USD_PATH,
            prim_path="/World/RoArm_M3"
        )
        
        # Position the arm
        execute("TransformPrimCommand",
            path="/World/RoArm_M3",
            translation=(0, 0, 0),
            rotation=(0, 0, 0, 1)
        )
        
        print(f"Imported RoArm-M3 Pro model from {ARM_USD_PATH}")
        return True
    else:
        print(f"USD model not found at {ARM_USD_PATH}")
        print("Creating simplified arm model instead")
        create_simplified_arm()
        return False

def setup_simulation():
    """Set up the simulation context."""
    # Create simulation context
    sim_context = SimulationContext()
    
    # Configure physics
    sim_context.set_physics_dt(SIMULATION_TIMESTEP)
    sim_context.set_rendering_dt(RENDERING_TIMESTEP)
    
    # Configure solver settings
    physics_scene = sim_context.get_physics_context().get_scene()
    physics_scene.set_solver_type("PGS")  # Projected Gauss-Seidel
    physics_scene.set_position_iteration_count(8)
    physics_scene.set_velocity_iteration_count(2)
    
    return sim_context

def get_arm_articulation():
    """Get the articulation view for the arm."""
    # Create articulation view
    arm_articulation = ArticulationView(
        prim_path="/World/RoArm_M3",
        name="roarm_articulation"
    )
    
    return arm_articulation

def run_simulation(sim_context, arm_articulation):
    """Run a simple simulation."""
    # Get articulation controller
    controller = arm_articulation.get_articulation_controller()
    
    # Print joint information
    num_joints = arm_articulation.num_joints
    print(f"RoArm-M3 Pro has {num_joints} joints")
    
    # Get joint positions
    joint_positions = arm_articulation.get_joint_positions()
    print(f"Initial joint positions: {joint_positions}")
    
    # Set target joint positions
    target_positions = np.zeros(num_joints)
    for i in range(num_joints):
        target_positions[i] = np.sin(i * np.pi / 4) * 0.5
    
    print(f"Target joint positions: {target_positions}")
    
    # Run simulation for 1000 steps (about 16.7 seconds at 60Hz)
    for i in range(1000):
        # Apply joint positions with sinusoidal motion
        t = i * SIMULATION_TIMESTEP
        positions = np.zeros(num_joints)
        for j in range(num_joints):
            positions[j] = np.sin(t + j * np.pi / 4) * 0.5
        
        # Apply action
        controller.apply_action(positions)
        
        # Step simulation
        sim_context.step()
        
        # Print progress every 100 steps
        if i % 100 == 0:
            current_positions = arm_articulation.get_joint_positions()
            print(f"Step {i}: Joint positions = {current_positions}")
    
    print("Simulation completed")

def main():
    """Main function."""
    print("Setting up basic simulation environment for RoArm-M3 Pro")
    
    # Create environment
    create_basic_environment()
    
    # Import or create arm model
    import_arm_model()
    
    # Set up simulation
    sim_context = setup_simulation()
    
    # Play the simulation
    sim_context.play()
    
    # Get arm articulation
    arm_articulation = get_arm_articulation()
    
    # Run simulation
    run_simulation(sim_context, arm_articulation)
    
    # Stop the simulation
    sim_context.stop()
    
    print("Example completed successfully")

if __name__ == "__main__":
    main()
