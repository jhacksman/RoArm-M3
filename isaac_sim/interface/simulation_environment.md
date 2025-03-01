# Setting Up the Simulation Environment for RoArm-M3 Pro

This guide explains how to create and configure a simulation environment in NVIDIA Isaac Sim for the RoArm-M3 Pro robotic arm.

## Overview

A well-designed simulation environment is crucial for:

1. Realistic testing of control algorithms
2. Generating synthetic data for training
3. Developing and validating robot behaviors
4. Sim-to-real transfer learning

## Basic Environment Setup

### Step 1: Create a New Stage

1. Launch Isaac Sim
2. Go to "File" > "New Stage"
3. Set up the stage properties:
   - Go to "Window" > "Stage Settings"
   - Set "Up Axis" to "Z"
   - Set appropriate physics settings (gravity, etc.)

### Step 2: Add a Ground Plane

```python
import omni.kit.commands
import omni.isaac.core.utils.prims as prim_utils

# Create ground plane
ground_prim = prim_utils.create_prim(
    "/World/ground",
    "Plane",
    attributes={"size": 10.0}
)

# Add physics properties
prim_utils.set_rigid_body_properties(
    ground_prim.GetPath(),
    mass=0,  # Static object
    linear_damping=0,
    angular_damping=0
)

# Add collision properties
prim_utils.set_collision_properties(
    ground_prim.GetPath(),
    collision_enabled=True,
    contact_offset=0.02,
    rest_offset=0.0
)
```

### Step 3: Add Lighting

```python
# Add dome light
light_prim = prim_utils.create_prim(
    "/World/Lights/DomeLight",
    "DomeLight"
)
prim_utils.set_attribute(light_prim.GetPath(), "intensity", 1000.0)
prim_utils.set_attribute(light_prim.GetPath(), "color", (1.0, 1.0, 1.0))

# Add key light
key_light = prim_utils.create_prim(
    "/World/Lights/KeyLight",
    "DistantLight"
)
prim_utils.set_attribute(key_light.GetPath(), "intensity", 2500.0)
prim_utils.set_attribute(key_light.GetPath(), "color", (1.0, 0.98, 0.95))
```

### Step 4: Import the RoArm-M3 Pro Model

If you've already created a model following the [Model Creation Guide](./model_creation.md):

```python
# Import existing USD model
omni.kit.commands.execute("CreateReferenceCommand",
    asset_path="/path/to/roarm_m3.usd",
    prim_path="/World/RoArm_M3"
)

# Position the arm
omni.kit.commands.execute("TransformPrimCommand",
    path="/World/RoArm_M3",
    translation=(0, 0, 0),
    rotation=(0, 0, 0, 1)
)
```

## Creating Task-Specific Environments

### Pick and Place Environment

```python
# Add a table
table_prim = prim_utils.create_prim(
    "/World/table",
    "Cube",
    attributes={"size": (0.8, 0.8, 0.02)}
)
omni.kit.commands.execute("TransformPrimCommand",
    path=table_prim.GetPath(),
    translation=(0.5, 0, 0.01)  # Position in front of the arm
)

# Add objects to manipulate
cube_prim = prim_utils.create_prim(
    "/World/objects/cube",
    "Cube",
    attributes={"size": (0.04, 0.04, 0.04)}
)
omni.kit.commands.execute("TransformPrimCommand",
    path=cube_prim.GetPath(),
    translation=(0.3, 0.1, 0.04)
)

# Add target location
target_prim = prim_utils.create_prim(
    "/World/target",
    "Cylinder",
    attributes={"radius": 0.05, "height": 0.001}
)
omni.kit.commands.execute("TransformPrimCommand",
    path=target_prim.GetPath(),
    translation=(0.3, -0.2, 0.02)
)
```

### Object Sorting Environment

```python
# Create sorting bins
bin1_prim = prim_utils.create_prim(
    "/World/bins/bin1",
    "Cube",
    attributes={"size": (0.2, 0.2, 0.05)}
)
omni.kit.commands.execute("TransformPrimCommand",
    path=bin1_prim.GetPath(),
    translation=(0.4, 0.3, 0.025)
)

bin2_prim = prim_utils.create_prim(
    "/World/bins/bin2",
    "Cube",
    attributes={"size": (0.2, 0.2, 0.05)}
)
omni.kit.commands.execute("TransformPrimCommand",
    path=bin2_prim.GetPath(),
    translation=(0.4, -0.3, 0.025)
)

# Create objects of different types
for i in range(5):
    # Create spheres (type 1)
    sphere_prim = prim_utils.create_prim(
        f"/World/objects/sphere_{i}",
        "Sphere",
        attributes={"radius": 0.02}
    )
    omni.kit.commands.execute("TransformPrimCommand",
        path=sphere_prim.GetPath(),
        translation=(0.3, 0.1 - i*0.05, 0.02)
    )
    
    # Create cubes (type 2)
    cube_prim = prim_utils.create_prim(
        f"/World/objects/cube_{i}",
        "Cube",
        attributes={"size": (0.04, 0.04, 0.04)}
    )
    omni.kit.commands.execute("TransformPrimCommand",
        path=cube_prim.GetPath(),
        translation=(0.3, -0.1 + i*0.05, 0.02)
    )
```

## Adding Sensors to the Environment

### Camera Sensor

```python
import omni.isaac.sensor as sensors

# Add camera
camera = sensors.Camera(
    prim_path="/World/camera",
    position=(1.0, 0.0, 0.5),
    rotation=(0.7071, 0, 0.7071, 0),  # Looking at the arm
    width=640,
    height=480
)

# Capture image
rgb_image = camera.get_rgba()
depth_image = camera.get_depth()

# Save images
import matplotlib.pyplot as plt
plt.imsave("camera_rgb.png", rgb_image)
```

### Depth Sensor

```python
# Add depth sensor
depth_sensor = sensors.DepthSensor(
    prim_path="/World/depth_sensor",
    position=(0.8, 0.0, 0.5),
    rotation=(0.7071, 0, 0.7071, 0),
    width=640,
    height=480
)

# Get depth data
depth_data = depth_sensor.get_depth_data()
```

## Physics Configuration

Configure physics properties for realistic simulation:

```python
# Configure physics settings
from omni.isaac.core.simulation_context import SimulationContext

sim_context = SimulationContext.instance()
sim_context.set_physics_dt(1.0 / 60.0)  # 60 Hz physics
sim_context.set_rendering_dt(1.0 / 30.0)  # 30 Hz rendering

# Configure solver settings
physics_scene = sim_context.get_physics_context().get_scene()
physics_scene.set_solver_type("PGS")  # Projected Gauss-Seidel
physics_scene.set_position_iteration_count(8)
physics_scene.set_velocity_iteration_count(2)
```

## Domain Randomization

Domain randomization helps bridge the reality gap by training on varied simulations:

```python
import numpy as np
import random

def randomize_environment(randomization_config):
    """Apply domain randomization to the environment.
    
    Args:
        randomization_config: Configuration for randomization
    """
    # Randomize lighting
    if "lighting" in randomization_config:
        # Get dome light
        dome_light_path = "/World/Lights/DomeLight"
        
        # Randomize intensity
        intensity_range = randomization_config["lighting"]["intensity"]
        intensity = np.random.uniform(intensity_range[0], intensity_range[1])
        prim_utils.set_attribute(dome_light_path, "intensity", intensity)
        
        # Randomize color
        if randomization_config["lighting"]["color"]:
            r = np.random.uniform(0.8, 1.0)
            g = np.random.uniform(0.8, 1.0)
            b = np.random.uniform(0.8, 1.0)
            prim_utils.set_attribute(dome_light_path, "color", (r, g, b))
    
    # Randomize object positions
    if "object_positions" in randomization_config:
        objects_path = "/World/objects"
        objects = prim_utils.get_all_matching_prims(objects_path)
        
        for obj in objects:
            # Get current position
            current_pos = prim_utils.get_prim_attribute(obj.GetPath(), "xformOp:translate")
            
            # Add random offset
            offset_range = randomization_config["object_positions"]
            x_offset = np.random.uniform(offset_range[0], offset_range[1])
            y_offset = np.random.uniform(offset_range[0], offset_range[1])
            
            # Set new position
            new_pos = (current_pos[0] + x_offset, current_pos[1] + y_offset, current_pos[2])
            prim_utils.set_attribute(obj.GetPath(), "xformOp:translate", new_pos)
    
    # Randomize textures
    if "textures" in randomization_config and randomization_config["textures"]:
        # List of available textures
        textures = [
            "/path/to/texture1.png",
            "/path/to/texture2.png",
            "/path/to/texture3.png"
        ]
        
        # Apply random textures to objects
        objects_path = "/World/objects"
        objects = prim_utils.get_all_matching_prims(objects_path)
        
        for obj in objects:
            # Select random texture
            texture = random.choice(textures)
            
            # Apply texture
            # (Implementation depends on Isaac Sim's material system)
            pass

# Example usage
randomization_config = {
    "lighting": {
        "intensity": [800, 1200],
        "color": True
    },
    "object_positions": [-0.05, 0.05],
    "textures": True
}

# Apply randomization before each training episode
randomize_environment(randomization_config)
```

## Calibrating Simulation to Match Reality

To ensure the simulation accurately represents the real world:

### Step 1: Collect Real-World Data

```python
def collect_real_world_data(bridge, trajectories):
    """Collect data from the physical robot.
    
    Args:
        bridge: Communication bridge to the physical robot
        trajectories: List of joint trajectories to execute
    
    Returns:
        dict: Collected data
    """
    data = {
        "joint_positions": [],
        "joint_velocities": [],
        "joint_torques": [],
        "timestamps": []
    }
    
    for trajectory in trajectories:
        for joint_positions in trajectory:
            # Send command to physical robot
            bridge.send_joint_positions(joint_positions)
            
            # Wait for movement to complete
            time.sleep(0.1)
            
            # Record data
            status = bridge.get_joint_status()
            
            data["joint_positions"].append(status["positions"])
            data["joint_velocities"].append(status["velocities"])
            data["joint_torques"].append(status["torques"])
            data["timestamps"].append(time.time())
    
    return data
```

### Step 2: Collect Simulation Data

```python
def collect_simulation_data(articulation, controller, trajectories):
    """Collect data from the simulation.
    
    Args:
        articulation: Isaac Sim articulation object
        controller: Articulation controller
        trajectories: List of joint trajectories to execute
    
    Returns:
        dict: Collected data
    """
    data = {
        "joint_positions": [],
        "joint_velocities": [],
        "joint_torques": [],
        "timestamps": []
    }
    
    sim_context = SimulationContext.instance()
    
    for trajectory in trajectories:
        for joint_positions in trajectory:
            # Send command to simulated robot
            controller.apply_action(joint_positions)
            
            # Step simulation
            for _ in range(10):  # 10 steps at 60Hz = ~0.17s
                sim_context.step()
            
            # Record data
            data["joint_positions"].append(articulation.get_joint_positions())
            data["joint_velocities"].append(articulation.get_joint_velocities())
            data["joint_torques"].append(articulation.get_joint_efforts())
            data["timestamps"].append(sim_context.current_time)
    
    return data
```

### Step 3: Compare and Calibrate

```python
def calculate_calibration_error(real_data, sim_data):
    """Calculate error between real and simulation data.
    
    Args:
        real_data: Data from the physical robot
        sim_data: Data from the simulation
    
    Returns:
        float: Error metric
    """
    # Calculate position error
    position_error = 0
    for real_pos, sim_pos in zip(real_data["joint_positions"], sim_data["joint_positions"]):
        position_error += np.mean(np.abs(np.array(real_pos) - np.array(sim_pos)))
    
    # Calculate velocity error
    velocity_error = 0
    for real_vel, sim_vel in zip(real_data["joint_velocities"], sim_data["joint_velocities"]):
        velocity_error += np.mean(np.abs(np.array(real_vel) - np.array(sim_vel)))
    
    # Calculate torque error
    torque_error = 0
    for real_torque, sim_torque in zip(real_data["joint_torques"], sim_data["joint_torques"]):
        torque_error += np.mean(np.abs(np.array(real_torque) - np.array(sim_torque)))
    
    # Combine errors
    total_error = position_error + 0.1 * velocity_error + 0.01 * torque_error
    
    return total_error

def optimize_simulation_parameters(articulation, real_data):
    """Optimize simulation parameters to match real data.
    
    Args:
        articulation: Isaac Sim articulation object
        real_data: Data from the physical robot
    
    Returns:
        dict: Optimized parameters
    """
    from scipy.optimize import minimize
    
    # Define parameters to optimize
    initial_params = {
        "mass_scales": [1.0] * articulation.num_links,
        "damping_scales": [1.0] * articulation.num_joints,
        "stiffness_scales": [1.0] * articulation.num_joints,
        "friction_scales": [1.0] * articulation.num_joints
    }
    
    # Flatten parameters for optimization
    initial_params_flat = []
    for param_list in initial_params.values():
        initial_params_flat.extend(param_list)
    
    # Define objective function
    def objective(params_flat):
        # Reconstruct parameter dictionary
        idx = 0
        params = {}
        for key, value in initial_params.items():
            params[key] = params_flat[idx:idx+len(value)]
            idx += len(value)
        
        # Apply parameters to simulation
        apply_parameters_to_sim(articulation, params)
        
        # Run simulation with the same trajectories
        sim_data = collect_simulation_data(
            articulation,
            articulation.get_articulation_controller(),
            real_data["trajectories"]
        )
        
        # Calculate error
        error = calculate_calibration_error(real_data, sim_data)
        
        return error
    
    # Run optimization
    result = minimize(
        objective,
        initial_params_flat,
        method="Nelder-Mead",
        options={"maxiter": 100}
    )
    
    # Reconstruct optimized parameters
    idx = 0
    optimized_params = {}
    for key, value in initial_params.items():
        optimized_params[key] = result.x[idx:idx+len(value)]
        idx += len(value)
    
    return optimized_params

def apply_parameters_to_sim(articulation, params):
    """Apply parameters to the simulation.
    
    Args:
        articulation: Isaac Sim articulation object
        params: Parameter dictionary
    """
    # Apply mass scales
    for i, scale in enumerate(params["mass_scales"]):
        current_mass = articulation.get_link_mass(i)
        articulation.set_link_mass(i, current_mass * scale)
    
    # Apply damping scales
    for i, scale in enumerate(params["damping_scales"]):
        current_damping = articulation.get_joint_damping(i)
        articulation.set_joint_damping(i, current_damping * scale)
    
    # Apply stiffness scales
    for i, scale in enumerate(params["stiffness_scales"]):
        current_stiffness = articulation.get_joint_stiffness(i)
        articulation.set_joint_stiffness(i, current_stiffness * scale)
    
    # Apply friction scales
    for i, scale in enumerate(params["friction_scales"]):
        current_friction = articulation.get_joint_friction(i)
        articulation.set_joint_friction(i, current_friction * scale)
```

## Performance Optimization

Optimize simulation performance for training:

### Memory and VRAM Considerations

```python
def optimize_memory_usage(sim_context):
    """Optimize memory usage for simulation.
    
    Args:
        sim_context: Isaac Sim simulation context
    """
    # Reduce rendering quality
    sim_context.set_rendering_dt(1.0 / 20.0)  # Lower rendering frequency
    
    # Disable shadows for performance
    render_product = sim_context.get_render_product()
    render_product.set_shadow_enabled(False)
    
    # Reduce texture resolution
    render_settings = sim_context.get_render_settings()
    render_settings.set_texture_resolution_scale(0.5)
    
    # Limit number of objects
    # (Implementation depends on your scene)
    
    # Monitor VRAM usage
    import torch
    print(f"CUDA memory allocated: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
    print(f"CUDA memory reserved: {torch.cuda.memory_reserved() / 1e9:.2f} GB")
```

### Parallel Simulations

```python
def create_parallel_environments(num_envs, base_scene_path):
    """Create multiple parallel simulation environments.
    
    Args:
        num_envs: Number of environments to create
        base_scene_path: Path to the base scene USD file
    
    Returns:
        list: List of environment objects
    """
    from omni.isaac.core.utils.stage import create_new_stage
    from omni.isaac.core.simulation_context import SimulationContext
    
    environments = []
    
    for i in range(num_envs):
        # Create new stage
        create_new_stage()
        
        # Load base scene
        omni.kit.commands.execute("CreateReferenceCommand",
            asset_path=base_scene_path,
            prim_path=f"/World_{i}"
        )
        
        # Create simulation context
        sim_context = SimulationContext()
        
        # Configure physics
        sim_context.set_physics_dt(1.0 / 60.0)
        
        # Add to list
        environments.append({
            "context": sim_context,
            "articulation": None  # Will be populated after stage is loaded
        })
        
        # Load articulation
        # (Implementation depends on your scene structure)
    
    return environments

def step_parallel_environments(environments, actions):
    """Step multiple environments in parallel.
    
    Args:
        environments: List of environment objects
        actions: List of actions for each environment
    
    Returns:
        list: Observations from each environment
    """
    observations = []
    
    for i, (env, action) in enumerate(zip(environments, actions)):
        # Apply action
        env["articulation"].get_articulation_controller().apply_action(action)
        
        # Step simulation
        env["context"].step()
        
        # Get observation
        observation = {
            "joint_positions": env["articulation"].get_joint_positions(),
            "joint_velocities": env["articulation"].get_joint_velocities()
        }
        
        observations.append(observation)
    
    return observations
```

## Saving and Loading Environments

```python
def save_environment(stage_path):
    """Save the current environment to a USD file.
    
    Args:
        stage_path: Path to save the USD file
    """
    omni.kit.commands.execute("SaveAs",
        file_path=stage_path,
        save_as_default=False
    )

def load_environment(stage_path):
    """Load an environment from a USD file.
    
    Args:
        stage_path: Path to the USD file
    """
    omni.kit.commands.execute("OpenStage",
        file_path=stage_path
    )
```

## Integration with LeRobot

To integrate the simulation environment with the LeRobot AI framework:

```python
def create_lerobot_compatible_environment():
    """Create a simulation environment compatible with LeRobot."""
    # Set up basic environment
    # ... (basic environment setup code)
    
    # Add observation interfaces
    # ... (add sensors, etc.)
    
    # Create LeRobot-compatible interface
    class LeRobotSimInterface:
        def __init__(self, articulation, sim_context):
            self.articulation = articulation
            self.sim_context = sim_context
            self.controller = articulation.get_articulation_controller()
        
        def get_observation(self):
            """Get observation in LeRobot format."""
            return {
                "joint_positions": self.articulation.get_joint_positions(),
                "joint_velocities": self.articulation.get_joint_velocities(),
                "end_effector_position": self._get_end_effector_position()
            }
        
        def apply_action(self, action):
            """Apply action from LeRobot."""
            self.controller.apply_action(action)
            self.sim_context.step()
        
        def _get_end_effector_position(self):
            """Get end effector position."""
            # Implementation depends on your articulation structure
            pass
    
    # Create and return interface
    return LeRobotSimInterface(roarm_articulation, sim_context)
```

## Next Steps

After setting up your simulation environment:

1. Proceed to [Sim-to-Real Transfer Learning](./sim_to_real.md)
2. Explore [Example Projects](../examples/README.md)
3. Learn about [Communication Interface](./communication.md)
