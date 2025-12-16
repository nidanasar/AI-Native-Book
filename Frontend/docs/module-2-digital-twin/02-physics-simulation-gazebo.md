---
sidebar_position: 2
title: "Chapter 2: Physics Simulation with Gazebo"
description: "Learn how to set up Gazebo for physics simulation with ROS 2 humanoid robots"
---

# Physics Simulation with Gazebo

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Set up a Gazebo simulation environment integrated with ROS 2
- Create simulation worlds using SDF (Simulation Description Format)
- Load and spawn URDF robot models into Gazebo
- Configure physics properties and observe realistic dynamics
- Control simulated joints through ROS 2 interfaces
:::

## Introduction to Gazebo

In Chapter 1, you learned that digital twins let robots rehearse actions safely. Now we'll set up **Gazebo**—the physics simulator that makes that rehearsal possible. Gazebo is to your robot what a mental practice space is to an athlete: a place to try movements, make mistakes, and refine techniques without real-world consequences.

### Gazebo Architecture

Modern Gazebo (also called "Ignition Gazebo" or "New Gazebo") is a complete rewrite of the classic Gazebo simulator. If you see references to "Gazebo Classic" or "gazebo11," those refer to the older version. This textbook uses **Gazebo Fortress** (paired with ROS 2 Humble) or **Gazebo Harmonic** (paired with ROS 2 Iron/Jazzy).

Gazebo consists of several components:

```
┌─────────────────────────────────────────────────────────────┐
│                     GAZEBO SIMULATOR                         │
├─────────────────┬─────────────────┬─────────────────────────┤
│   GUI Client    │  Physics Engine │    Sensor Plugins       │
│   (gz gui)      │  (DART/ODE)     │  (cameras, LiDAR, etc.) │
├─────────────────┴─────────────────┴─────────────────────────┤
│                    Gazebo Transport                          │
│            (internal message passing system)                 │
├─────────────────────────────────────────────────────────────┤
│                       gz sim                                 │
│             (simulation server - the core)                   │
└─────────────────────────────────────────────────────────────┘
```

Key components:

- **gz sim**: The simulation server that runs physics and manages the world
- **Physics Engine**: Computes forces, collisions, and motion (typically DART)
- **Sensor Plugins**: Generate simulated sensor data (covered in Chapter 3)
- **GUI Client**: Visualization interface for watching the simulation

### ROS 2 Integration via ros_gz_bridge

Gazebo has its own transport system, but we want to use ROS 2 topics. The **ros_gz_bridge** package translates between them:

```
┌─────────────┐                          ┌─────────────┐
│   ROS 2     │    ros_gz_bridge         │   Gazebo    │
│   Node      │◄────────────────────────►│   Plugin    │
└─────────────┘                          └─────────────┘
      │                                         │
      │  /cmd_vel (ROS 2 topic)                │  gz transport
      │  /joint_states (ROS 2 topic)           │  (internal)
      ▼                                         ▼
Your control code                        Physics simulation
```

This bridge means your ROS 2 code doesn't know (or care) whether it's talking to a real robot or a simulated one. The same code works for both—exactly as the middleware abstraction intended.

:::tip Key Insight
Think of ros_gz_bridge as a translator between two languages. Your robot "speaks" ROS 2, Gazebo "speaks" gz transport, and the bridge ensures they understand each other.
:::

### Installing Gazebo with ROS 2

For ROS 2 Humble, install Gazebo Fortress:

```bash
# Install Gazebo Fortress and ROS 2 integration packages
sudo apt install ros-humble-ros-gz

# This installs:
# - gz-fortress (the simulator)
# - ros_gz_bridge (ROS 2 <-> Gazebo bridge)
# - ros_gz_sim (launch file helpers)
```

For ROS 2 Iron or Jazzy, install Gazebo Harmonic:

```bash
sudo apt install ros-iron-ros-gz  # or ros-jazzy-ros-gz
```

Verify your installation:

```bash
# Check Gazebo version
gz sim --version

# Start an empty simulation
gz sim empty.sdf
```

## Creating Simulation Worlds

A Gazebo **world** defines the environment where your robot operates. Worlds are described in **SDF (Simulation Description Format)**—an XML format similar to URDF but designed for entire environments, not just robots.

### SDF vs URDF

You already know URDF from Module 1. Here's how SDF differs:

| Aspect | URDF | SDF |
|--------|------|-----|
| Purpose | Robot description | World + robot description |
| Scope | Single robot model | Entire simulation environment |
| Features | Links, joints, sensors | Physics settings, lighting, multiple models |
| ROS Integration | Native | Via conversion or direct loading |

SDF is a superset—it can contain URDF-like robot models plus environmental elements.

### Basic World Structure

Here's a minimal world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="simple_world">

    <!-- Physics configuration -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

Key elements:

- **`<physics>`**: Configures the physics engine (timestep, real-time factor)
- **`<light>`**: Adds illumination for rendering
- **`<model>`**: Defines objects in the world (ground plane, obstacles, robots)

### Adding Objects to the World

You can add obstacles for your robot to interact with:

```xml
<!-- A box obstacle -->
<model name="box_obstacle">
  <pose>2 0 0.5 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <link name="link">
    <inertial>
      <mass>10.0</mass>
      <inertia>
        <ixx>0.83</ixx><iyy>0.83</iyy><izz>0.83</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <ambient>0.3 0.3 0.8 1</ambient>
      </material>
    </visual>
  </link>
</model>
```

### World Files for Humanoid Testing

For humanoid robots, useful world configurations include:

| World Type | Description | Use Case |
|-----------|-------------|----------|
| Flat ground | Simple plane | Basic standing/walking |
| Stairs | Step sequence | Stair climbing tests |
| Uneven terrain | Bumps and slopes | Balance testing |
| Indoor room | Walls, furniture | Navigation testing |

:::warning Common Mistake
Don't forget to add gravity! Without the physics plugin or proper configuration, your robot will float. The default DART physics engine includes Earth gravity (-9.81 m/s² in Z).
:::

## Spawning Your Robot

With a world ready, the next step is adding your robot. Gazebo can load URDF models directly or convert them to SDF.

### Method 1: Spawn via ROS 2 Service

The recommended approach uses the `ros_gz_sim` package:

```bash
# Terminal 1: Start Gazebo with ROS 2 bridge
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"

# Terminal 2: Spawn your robot
ros2 run ros_gz_sim create \
    -name simple_humanoid \
    -topic robot_description \
    -z 1.0
```

This spawns a robot from the `/robot_description` topic (published by `robot_state_publisher`) at height z=1.0 meters.

### Method 2: Include in World File

You can include the robot directly in your SDF world:

```xml
<include>
  <uri>model://simple_humanoid</uri>
  <pose>0 0 1.0 0 0 0</pose>
</include>
```

This requires your robot model to be in Gazebo's model path.

### Launch File for Complete Setup

A typical launch file combines everything:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to your URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_humanoid_description'),
        'urdf', 'simple_humanoid.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Path to your world file
    world_file = os.path.join(
        get_package_share_directory('my_humanoid_gazebo'),
        'worlds', 'simple_world.sdf'
    )

    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': world_file}.items(),
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'simple_humanoid',
                '-topic', 'robot_description',
                '-z', '1.0',
            ],
            output='screen',
        ),

        # Bridge between Gazebo and ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen',
        ),
    ])
```

### Verifying the Spawn

After launching, verify your robot loaded correctly:

```bash
# List models in Gazebo
gz model --list

# Check ROS 2 topics
ros2 topic list | grep joint

# Echo joint states
ros2 topic echo /joint_states
```

You should see your robot in the Gazebo GUI and joint states publishing on ROS 2 topics.

## Physics Properties and Dynamics

Gazebo simulates physics based on properties you define. Understanding these helps you create realistic simulations.

### Gravity

By default, Gazebo applies Earth gravity (9.81 m/s² downward). You can modify this:

```xml
<physics name="default_physics" type="dart">
  <gravity>0 0 -9.81</gravity>  <!-- Earth gravity -->
  <!-- For Moon: <gravity>0 0 -1.62</gravity> -->
  <!-- For Mars: <gravity>0 0 -3.71</gravity> -->
</physics>
```

### Friction

Friction determines how surfaces interact. Configure it in collision elements:

```xml
<collision name="foot_collision">
  <geometry>
    <box><size>0.15 0.08 0.02</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>      <!-- Friction coefficient (0-1+) -->
        <mu2>0.8</mu2>    <!-- Secondary friction -->
      </ode>
    </friction>
  </surface>
</collision>
```

Higher friction values (closer to 1.0) make surfaces "grippy"—important for humanoid feet!

### Inertia

Inertia determines how objects resist rotational acceleration. From Module 1, you know URDF includes inertia tensors:

```xml
<inertial>
  <mass value="5.0"/>
  <inertia
    ixx="0.1" ixy="0" ixz="0"
    iyy="0.1" iyz="0"
    izz="0.05"/>
</inertial>
```

:::warning Common Issue
Incorrect inertia values cause unstable simulation. If your robot vibrates or explodes, check inertia values. A good approximation for a box: `I = (1/12) × mass × (width² + height²)`.
:::

### Physics Engine Options

Gazebo supports multiple physics engines:

| Engine | Strengths | Best For |
|--------|-----------|----------|
| DART | Contact handling, constraints | Humanoid robots, manipulation |
| ODE | Speed, stability | Ground vehicles, simple robots |
| Bullet | Game-like physics | Quick prototyping |

DART is recommended for humanoid robots due to superior contact handling—crucial for walking and balance.

### Timestep and Real-Time Factor

Two important physics parameters:

```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.001</max_step_size>   <!-- 1ms timestep -->
  <real_time_factor>1.0</real_time_factor> <!-- 1.0 = real-time -->
</physics>
```

- **max_step_size**: Smaller = more accurate but slower. 1ms works well for humanoids.
- **real_time_factor**: Set greater than 1.0 for faster-than-real-time (useful for training), less than 1.0 for slow motion.

## Controlling Simulated Joints

With your robot spawned and physics configured, you need to control it. Gazebo integrates with ROS 2 control systems through plugins.

### Joint State Publisher

To read joint positions from Gazebo, use the JointStatePub plugin:

```xml
<!-- Add to your robot model in SDF or URDF (Gazebo-specific) -->
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
    <topic>joint_states</topic>
  </plugin>
</gazebo>
```

This publishes joint positions, velocities, and efforts to a Gazebo topic, which `ros_gz_bridge` forwards to ROS 2.

### Joint Position Controller

To command joint positions:

```xml
<gazebo>
  <plugin filename="gz-sim-joint-position-controller-system"
          name="gz::sim::systems::JointPositionController">
    <joint_name>left_shoulder_joint</joint_name>
    <topic>left_shoulder_cmd</topic>
    <p_gain>100</p_gain>
    <i_gain>0.1</i_gain>
    <d_gain>10</d_gain>
  </plugin>
</gazebo>
```

Then send commands via ROS 2 (after bridging):

```bash
ros2 topic pub /left_shoulder_cmd std_msgs/msg/Float64 "data: 0.5"
```

### ros2_control Integration

For complex robots, **ros2_control** provides a standardized control framework:

```xml
<!-- In your URDF -->
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="left_shoulder_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... more joints ... -->
</ros2_control>
```

This integrates with ROS 2 controllers like `joint_trajectory_controller` for smooth motion:

```bash
# Load and configure controllers
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_trajectory_controller active
```

### Observing Behavior

Monitor your simulation through various tools:

```bash
# Watch joint states in terminal
ros2 topic echo /joint_states

# Visualize in RViz
ros2 run rviz2 rviz2

# Plot joint positions over time
ros2 run rqt_plot rqt_plot /joint_states/position[0]
```

The Gazebo GUI also shows real-time information:
- Select your robot model
- View → Component Inspector shows pose, velocity, forces

## Summary

In this chapter, you learned how to create physics simulations for humanoid robots using Gazebo:

**Key Concepts:**

- **Gazebo** is a physics simulator that integrates with ROS 2 via ros_gz_bridge
  - New Gazebo (Fortress/Harmonic) replaces Gazebo Classic
  - Uses DART physics engine for accurate humanoid simulation

- **World files (SDF)** define the simulation environment
  - Configure physics, lighting, ground, obstacles
  - SDF is similar to URDF but for entire worlds

- **Spawning robots** can be done via:
  - ROS 2 service (`ros_gz_sim create`)
  - Direct inclusion in world file
  - Launch files for complete setup

- **Physics properties** affect simulation realism:
  - Gravity, friction, inertia
  - Timestep and real-time factor
  - Choose appropriate physics engine (DART for humanoids)

- **Joint control** through Gazebo plugins:
  - JointStatePublisher for reading states
  - JointPositionController for commanding
  - ros2_control integration for complex behaviors

**The Virtual Rehearsal Continues:**

| Concept | Analogy |
|---------|---------|
| Gazebo world | The mental "stage" where rehearsal happens |
| Physics engine | The brain's internal model of how things move |
| Spawning robot | Imagining yourself in the scenario |
| Joint control | Mentally commanding your body to move |

**What's Next:**

Your robot can now move in simulation, but it's blind and deaf—it has no sensors. In Chapter 3, we'll add simulated LiDAR, cameras, and IMUs so your robot can perceive its virtual environment, completing the sensory feedback loop that makes digital twins truly useful for developing perception and control algorithms.

---

:::tip Next Chapter
Continue to [Chapter 3: Simulating Sensors](./simulating-sensors) to learn how to add LiDAR, depth cameras, and IMUs to your simulated humanoid.
:::
