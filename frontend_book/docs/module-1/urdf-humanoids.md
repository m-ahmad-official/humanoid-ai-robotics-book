---
sidebar_position: 3
title: Robot Structure with URDF
---

# Robot Structure with URDF

## Purpose of URDF in Humanoid Robotics

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. In humanoid robotics, URDF plays a crucial role in defining the physical structure, kinematic properties, and visual appearance of humanoid robots.

### Key Benefits of URDF in Humanoid Robotics

1. **Kinematic Modeling**: Defines the kinematic chain of the humanoid robot, including all joints and links
2. **Simulation Compatibility**: Enables accurate simulation of humanoid robots in Gazebo and other simulators
3. **Visualization**: Provides visual representation for tools like RViz
4. **Collision Detection**: Defines collision properties for safe robot operation
5. **Motion Planning**: Enables trajectory planning and inverse kinematics calculations

## Links, Joints, and Kinematic Chains

### Links

A **link** in URDF represents a rigid body part of the robot. Each link has:

- **Inertial properties**: Mass, center of mass, and inertia tensor
- **Visual properties**: How the link appears in simulation and visualization
- **Collision properties**: How the link interacts with other objects for collision detection

#### Example Link Definition:

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.1" length="0.2" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.1" length="0.2" />
    </geometry>
  </collision>
</link>
```

### Joints

A **joint** connects two links and defines the allowed motion between them. Joint types include:

- **Fixed**: No motion allowed (rigid connection)
- **Revolute**: Single axis rotation with limits
- **Continuous**: Single axis rotation without limits
- **Prismatic**: Single axis translation with limits
- **Floating**: 6DOF motion (rarely used)
- **Planar**: Motion in a plane

#### Example Joint Definition:

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link" />
  <child link="wheel_link" />
  <origin xyz="0.1 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
</joint>
```

### Kinematic Chains

Kinematic chains in humanoid robots typically include:

- **Torso chain**: From base to head
- **Arm chains**: Left and right arms from shoulder to hand
- **Leg chains**: Left and right legs from hip to foot

## Preparing Humanoid Robots for Simulation and Control

### URDF for Simulation

To prepare a humanoid robot for simulation, the URDF must include:

1. **Accurate inertial properties**: For realistic physics simulation
2. **Collision geometries**: For collision detection
3. **Joint limits**: To prevent impossible movements
4. **Transmission elements**: To connect joints with actuators

#### Example Complete Robot URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
    </collision>
  </link>

  <!-- Head Link -->
  <link name="head_link">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link" />
    <child link="head_link" />
    <origin xyz="0 0 0.2" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="0.3" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.03" length="0.2" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="base_link" />
    <child link="left_upper_arm" />
    <origin xyz="0.15 0 0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>
</robot>
```

### URDF for Control

For robot control, the URDF must also include:

1. **Transmission elements**: Define how actuators connect to joints
2. **Joint properties**: Include friction, damping, and other physical properties
3. **Effort/velocity limits**: Ensure safe operation

#### Example Transmission:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## URDF's Role in ROS 2 and Simulators

### Integration with ROS 2

URDF integrates with ROS 2 through several key components:

1. **Robot State Publisher**: Publishes transforms based on joint states
2. **TF2**: Provides coordinate transformations between links
3. **RViz**: Visualizes the robot model
4. **MoveIt**: Uses URDF for motion planning

### ROS 2 Launch Files

URDF files are typically loaded in ROS 2 launch files:

```python
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        get_package_share_directory('my_robot_description'),
        'urdf',
        'robot.urdf'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    return LaunchDescription([
        robot_state_publisher
    ])
```

### Simulation in Gazebo

For Gazebo simulation, URDF may need additional Gazebo-specific tags:

```xml
<gazebo reference="left_wheel_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>

<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
  </plugin>
</gazebo>
```

## Best Practices for Humanoid Robot URDF

### 1. Proper Scaling
- Use consistent units (meters for length, kilograms for mass)
- Ensure realistic proportions for the intended robot

### 2. Accurate Inertial Properties
- Calculate or measure actual inertial properties
- Use CAD software to export accurate values

### 3. Appropriate Joint Limits
- Set realistic limits based on physical constraints
- Include safety margins to prevent damage

### 4. Collision Geometry Optimization
- Use simple shapes for collision detection (spheres, boxes, cylinders)
- Balance accuracy with computational efficiency

### 5. Hierarchical Structure
- Organize links and joints in a logical hierarchy
- Use meaningful names that reflect the robot's anatomy

## Example: Complete Humanoid Robot URDF

Here's a more complete example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include common properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.55 0.0 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.87 0.84 0.71 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.2 0.2" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.2 0.2" />
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link" />
    <child link="head" />
    <origin xyz="0 0 0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.2" />
      </geometry>
      <material name="blue" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.2" />
      </geometry>
    </collision>
  </link>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.3" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.03" length="0.2" />
      </geometry>
      <material name="blue" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.03" length="0.2" />
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link" />
    <child link="left_upper_arm" />
    <origin xyz="0.15 0 0.2" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm" />
    <child link="left_lower_arm" />
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>

  <!-- Right Arm (similar to left) -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.2" />
      </geometry>
      <material name="blue" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.2" />
      </geometry>
    </collision>
  </link>

  <link name="right_lower_arm">
    <inertial>
      <mass value="0.3" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.03" length="0.2" />
      </geometry>
      <material name="blue" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.03" length="0.2" />
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link" />
    <child link="right_upper_arm" />
    <origin xyz="-0.15 0 0.2" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm" />
    <child link="right_lower_arm" />
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <inertial>
      <mass value="0.8" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.003" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.3" />
      </geometry>
      <material name="red" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.3" />
      </geometry>
    </collision>
  </link>

  <link name="left_lower_leg">
    <inertial>
      <mass value="0.6" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.3" />
      </geometry>
      <material name="red" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.3" />
      </geometry>
    </collision>
  </link>

  <link name="left_foot">
    <inertial>
      <mass value="0.2" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.08 0.05" />
      </geometry>
      <material name="black" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.08 0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="left_upper_leg" />
    <origin xyz="0.07 0 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg" />
    <child link="left_lower_leg" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg" />
    <child link="left_foot" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0" />
  </joint>

  <!-- Right Leg (similar to left) -->
  <link name="right_upper_leg">
    <inertial>
      <mass value="0.8" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.003" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.3" />
      </geometry>
      <material name="red" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.3" />
      </geometry>
    </collision>
  </link>

  <link name="right_lower_leg">
    <inertial>
      <mass value="0.6" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.3" />
      </geometry>
      <material name="red" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.04" length="0.3" />
      </geometry>
    </collision>
  </link>

  <link name="right_foot">
    <inertial>
      <mass value="0.2" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.08 0.05" />
      </geometry>
      <material name="black" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.08 0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="right_upper_leg" />
    <origin xyz="-0.07 0 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg" />
    <child link="right_lower_leg" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg" />
    <child link="right_foot" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0" />
  </joint>
</robot>
```

## Summary

URDF is fundamental to humanoid robotics in ROS 2, providing the necessary description for simulation, visualization, and control. Understanding links, joints, and kinematic chains is essential for creating effective humanoid robot models. Properly structured URDF files enable accurate simulation and safe control of complex humanoid robots.