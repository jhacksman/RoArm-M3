# Creating a Digital Twin of the RoArm-M3 Pro in Isaac Sim

This guide explains how to create an accurate digital twin of the RoArm-M3 Pro robotic arm in NVIDIA Isaac Sim.

## Overview

Creating a digital twin involves:

1. Importing or creating 3D models of the arm components
2. Setting up proper articulations and joint configurations
3. Defining physical properties and constraints
4. Configuring sensors and actuators
5. Validating the model's behavior

## Prerequisites

Before creating the model, ensure you have:

- NVIDIA Isaac Sim installed and configured (see [Installation Guide](./installation.md))
- Basic understanding of USD (Universal Scene Description) format
- 3D models of the RoArm-M3 Pro components (optional, we'll provide alternatives)
- RoArm-M3 Pro specifications and measurements

## Method 1: Using Existing 3D Models

If you have access to 3D models of the RoArm-M3 Pro (e.g., STEP, STL, or OBJ files):

### Step 1: Convert Models to USD Format

1. **Install the necessary tools**:
   ```bash
   pip install usd-core
   ```

2. **Convert STL/OBJ files to USD**:
   ```bash
   # For STL files
   usdcat input.stl -o output.usd
   
   # For OBJ files
   usdcat input.obj -o output.usd
   ```

   Alternatively, use Blender with the USD exporter plugin.

### Step 2: Import Models into Isaac Sim

1. Launch Isaac Sim
2. Go to "File" > "Import" and select your USD files
3. Arrange the components in the correct positions

## Method 2: Creating a Model from Scratch

If you don't have 3D models, you can create a simplified representation:

### Step 1: Create Basic Shapes

1. Launch Isaac Sim
2. Create a new stage: "File" > "New Stage"
3. Add primitive shapes for each arm component:
   - Base: Cylinder
   - Arm segments: Elongated cubes
   - Joints: Spheres or cylinders
   - End effector: Custom shape based on the gripper

4. **Python Script for Basic Model Creation**:
   ```python
   import omni.kit.commands
   import omni.isaac.core.utils.prims as prim_utils
   
   # Create base
   base_prim = prim_utils.create_prim(
       "/World/RoArm_M3/base",
       "Cylinder",
       attributes={"radius": 0.1, "height": 0.05}
   )
   
   # Create first arm segment
   arm1_prim = prim_utils.create_prim(
       "/World/RoArm_M3/arm1",
       "Cube",
       attributes={"size": (0.05, 0.05, 0.2)}
   )
   
   # Position the first arm segment
   omni.kit.commands.execute("TransformPrimCommand",
       path=arm1_prim.GetPath(),
       translation=(0, 0, 0.15)
   )
   
   # Continue for other components...
   ```

### Step 2: Define Articulations

Create proper articulations for the joints:

1. Select the base component
2. Right-click and select "Create" > "Physics" > "Articulation Root"
3. For each joint:
   - Select the parent component
   - Right-click and select "Create" > "Physics" > "Articulation Body"
   - Configure the joint type and limits

4. **Python Script for Articulations**:
   ```python
   import omni.isaac.core.articulations as articulations
   
   # Create articulation
   articulation = articulations.ArticulationView(
       prim_path="/World/RoArm_M3",
       name="roarm_articulation"
   )
   
   # Configure joints
   joint_paths = [
       "/World/RoArm_M3/base/joint1",
       "/World/RoArm_M3/arm1/joint2",
       # Add other joints...
   ]
   
   # Set joint properties
   for joint_path in joint_paths:
       # Set joint type (revolute for rotational joints)
       prim_utils.set_attribute(joint_path, "type", "revolute")
       
       # Set joint limits
       prim_utils.set_attribute(joint_path, "lowerLimit", -3.14)
       prim_utils.set_attribute(joint_path, "upperLimit", 3.14)
       
       # Set drive properties
       prim_utils.set_attribute(joint_path, "driveType", "position")
       prim_utils.set_attribute(joint_path, "stiffness", 1000.0)
       prim_utils.set_attribute(joint_path, "damping", 100.0)
   ```

## Method 3: Using URDF (Unified Robot Description Format)

If you have a URDF description of the RoArm-M3 Pro:

### Step 1: Create or Obtain URDF

Create a URDF file describing the RoArm-M3 Pro structure:

```xml
<?xml version="1.0"?>
<robot name="roarm_m3">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- First joint -->
  <joint name="base_to_arm1" type="revolute">
    <parent link="base_link"/>
    <child link="arm1_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- First arm segment -->
  <link name="arm1_link">
    <!-- Define visual, collision, and inertial properties -->
  </link>
  
  <!-- Continue with other joints and links... -->
</robot>
```

### Step 2: Import URDF into Isaac Sim

1. Launch Isaac Sim
2. Use the URDF importer:
   ```python
   import omni.isaac.urdf as urdf
   
   # Create importer
   urdf_interface = urdf.acquire_urdf_interface()
   
   # Import URDF
   result, articulation_path = urdf_interface.import_urdf(
       urdf_path="/path/to/roarm_m3.urdf",
       import_config=urdf.ImportConfig()
   )
   
   print(f"Imported articulation at: {articulation_path}")
   ```

## Configuring Physical Properties

After creating the basic model, configure physical properties:

### Step 1: Set Mass and Inertia

For each component:

1. Select the component
2. In the Property panel, find "Physics" > "Mass Properties"
3. Set appropriate mass and inertia values

### Step 2: Configure Joint Properties

For each joint:

1. Select the joint
2. Configure:
   - Joint type (revolute for rotational joints)
   - Joint limits (min/max angles)
   - Drive properties (stiffness, damping)

### Step 3: Add Materials

1. Create or import materials
2. Assign materials to components for visual appearance
3. Configure friction and restitution properties

## Adding Sensors

Add virtual sensors to match the physical arm's capabilities:

### Position Sensors

```python
import omni.isaac.sensor as sensors

# Add joint position sensor
position_sensor = sensors.JointPositionSensor(
    articulation_path="/World/RoArm_M3",
    joint_indices=[0, 1, 2, 3, 4, 5]  # Indices of joints to monitor
)

# Read sensor data
positions = position_sensor.get_joint_positions()
print(f"Joint positions: {positions}")
```

### Force/Torque Sensors

```python
# Add force/torque sensor
force_sensor = sensors.ForceSensor(
    prim_path="/World/RoArm_M3/end_effector/force_sensor",
    name="gripper_force_sensor"
)

# Read sensor data
force_data = force_sensor.get_force_data()
print(f"Force data: {force_data}")
```

## Validating the Model

Test the model to ensure it behaves like the physical arm:

### Step 1: Basic Movement Test

```python
import omni.isaac.core.utils.rotations as rot_utils
import numpy as np

# Get articulation controller
controller = articulation.get_articulation_controller()

# Set joint positions
target_positions = np.array([0.5, 0.3, 0.2, 0.1, 0.0, 0.0])
controller.apply_action(target_positions)

# Simulate for a few steps
for _ in range(100):
    simulation_context.step(render=True)
    
# Get resulting positions
current_positions = articulation.get_joint_positions()
print(f"Target: {target_positions}")
print(f"Actual: {current_positions}")
```

### Step 2: Workspace Validation

Create a script to test the arm's range of motion:

```python
import numpy as np
import matplotlib.pyplot as plt
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# Test points in joint space
test_points = []
for angle1 in np.linspace(-3.14, 3.14, 10):
    for angle2 in np.linspace(-1.57, 1.57, 10):
        test_points.append([angle1, angle2, 0, 0, 0, 0])

# Record end effector positions
end_effector_positions = []
for point in test_points:
    controller.apply_action(point)
    
    # Simulate for a few steps
    for _ in range(50):
        simulation_context.step(render=False)
    
    # Get end effector position
    end_effector_pos = articulation.get_world_pose(
        "/World/RoArm_M3/end_effector"
    )[0]
    end_effector_positions.append(end_effector_pos)

# Visualize workspace
x = [pos[0] for pos in end_effector_positions]
y = [pos[1] for pos in end_effector_positions]
z = [pos[2] for pos in end_effector_positions]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)
plt.savefig("workspace.png")
```

## Exporting the Model

Save your model for future use:

1. Go to "File" > "Save As" and save as a USD file
2. For reusable components, consider saving as USD components

## Next Steps

After creating your digital twin:

1. Proceed to [Setting up the Simulation Environment](./simulation_environment.md)
2. Learn how to [Connect Isaac Sim to the Physical Arm](./communication.md)
3. Explore [Sim-to-Real Transfer Learning](./sim_to_real.md)
