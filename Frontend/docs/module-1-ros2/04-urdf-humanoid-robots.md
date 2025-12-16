---
sidebar_position: 4
title: "Chapter 4: Understanding URDF for Humanoid Robots"
description: "Learn how humanoid robot bodies are described using URDF"
---

# Understanding URDF for Humanoid Robots

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Explain what URDF is and why it matters for robotics
- Identify links and joints in a URDF file
- Trace the kinematic chain of a humanoid robot
- Explain how URDF connects to ROS 2 via robot_state_publisher
:::

## What is URDF?

In the previous chapters, you learned how ROS 2 nodes communicate through topics and services—the robot's "nervous system." But a nervous system needs a body to control. **URDF (Unified Robot Description Format)** is how we define that body in software.

### The Robot's Anatomical Blueprint

URDF is an XML format that describes a robot's physical structure:

- What parts (links) the robot has
- How those parts connect (joints)
- The shape and size of each part (geometry)
- Physical properties like mass and inertia

Think of URDF as the robot's anatomical blueprint. Just as medical textbooks describe the human skeleton with bones and joints, URDF describes the robot skeleton with links and joints. The nervous system (ROS 2) reads this blueprint to understand what it's controlling.

### Why URDF Matters

Without URDF, the robot's software wouldn't know:

| Question | URDF Provides |
|----------|---------------|
| How many joints exist? | Complete joint list with names |
| How do joints move? | Rotation axes and limits |
| Where is each part located? | Coordinate frames and transforms |
| What does the robot look like? | Visual geometry for displays |
| How does the robot interact physically? | Collision geometry for simulation |

This information is essential for:

- **Motion planning**: Calculating how to move from pose A to pose B
- **Visualization**: Displaying the robot in RViz or other tools
- **Simulation**: Running the robot in Gazebo before deploying to hardware
- **Control**: Knowing joint limits to prevent damage

### URDF in the Nervous System Analogy

Extending our nervous system metaphor:

| Human Body | URDF Equivalent |
|------------|-----------------|
| Skeleton | The complete URDF file |
| Bones | Links (rigid body segments) |
| Joints | Joints (articulation points) |
| Proprioception | robot_state_publisher + TF |

Your brain has a "body map"—an internal representation of where your limbs are in space. URDF gives ROS 2 the same capability. The robot knows its own structure and can track where each part is as joints move.

:::tip Key Insight
URDF defines the robot's structure *statically*. The dynamic state (actual joint positions) comes from sensors and is published on the `/joint_states` topic. Together, structure + state = complete body awareness.
:::

## Links and Joints Explained

The two fundamental building blocks of URDF are **links** and **joints**. Understanding these is essential for reading and writing robot descriptions.

### Links: The Rigid Bodies

A **link** is a rigid body segment—a part of the robot that doesn't deform. In a humanoid:

- Torso is a link
- Upper arm is a link
- Forearm is a link
- Head is a link

Each link has three main properties:

#### 1. Visual Geometry

What the link looks like for display purposes:

```xml
<visual>
  <geometry>
    <box size="0.3 0.2 0.5"/>  <!-- width, depth, height -->
  </geometry>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <material name="blue"/>
</visual>
```

Common geometry types:
- `<box size="x y z"/>` — Rectangular box
- `<cylinder radius="r" length="l"/>` — Cylinder
- `<sphere radius="r"/>` — Sphere
- `<mesh filename="path/to/mesh.stl"/>` — Custom 3D model

#### 2. Collision Geometry

What the link uses for physics simulation (often simpler than visual):

```xml
<collision>
  <geometry>
    <box size="0.3 0.2 0.5"/>
  </geometry>
</collision>
```

#### 3. Inertial Properties

Mass and moment of inertia for dynamics simulation:

```xml
<inertial>
  <mass value="5.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
</inertial>
```

### Joints: The Articulation Points

A **joint** connects two links and defines how they can move relative to each other. Every joint has:

- A **parent link** (the link it attaches from)
- A **child link** (the link it attaches to)
- A **type** defining the motion

#### Joint Types

| Type | Motion | DOF | Example |
|------|--------|-----|---------|
| `revolute` | Rotation with limits | 1 | Elbow, knee |
| `continuous` | Unlimited rotation | 1 | Wheel |
| `prismatic` | Linear sliding | 1 | Telescope, drawer |
| `fixed` | No motion | 0 | Sensor mount |
| `floating` | All motion | 6 | Base in simulation |
| `planar` | 2D plane motion | 2 | XY table |

For humanoid robots, **revolute** joints are most common—they rotate like your elbows and shoulders.

#### Joint Definition Example

```xml
<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_lower_arm"/>
  <origin xyz="0 0 -0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="15" velocity="2.0"/>
</joint>
```

Key elements:

- **`<origin>`**: Where the joint is located relative to the parent link
- **`<axis>`**: Which direction the joint rotates around (x, y, or z)
- **`<limit>`**: Range of motion and force/speed limits

:::warning Common Mistake
The `<axis>` element specifies the rotation axis in the joint's frame. `xyz="0 1 0"` means rotation around the Y-axis. Getting this wrong causes the robot to move in unexpected directions.
:::

### The Kinematic Chain

Links and joints form a **kinematic chain**—a tree structure starting from a root link (usually called `base_link`). Each link is a child of exactly one parent through a joint.

```
base_link (torso)
├── neck_joint → head
├── left_shoulder_joint → left_upper_arm
│   └── left_elbow_joint → left_lower_arm
└── right_shoulder_joint → right_upper_arm
    └── right_elbow_joint → right_lower_arm
```

This hierarchical structure means:
- Moving the torso moves everything attached to it
- Moving the shoulder moves the entire arm
- Moving the elbow only moves the forearm

The same principle applies to your body—your hand position depends on your shoulder, elbow, and wrist positions combined.

## Building a Simple Humanoid Description

Let's examine a complete URDF for a simplified humanoid robot. This model has a torso, head, and two arms with shoulder and elbow joints—6 degrees of freedom total.

### Complete URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Material definitions for colors -->
  <material name="blue">
    <color rgba="0.2 0.4 0.8 1.0"/>
  </material>
  <material name="orange">
    <color rgba="0.9 0.5 0.2 1.0"/>
  </material>

  <!-- Base Link (Torso) - The root of our kinematic tree -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.55" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw: turn head left/right -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="orange"/>
    </visual>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.45" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch: swing arm forward/back -->
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>

  <!-- Left Lower Arm -->
  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.22"/>
      </geometry>
      <origin xyz="0 0 -0.11" rpy="0 0 0"/>
      <material name="orange"/>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Left Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="15" velocity="2.0"/>
  </joint>

  <!-- Right arm mirrors left arm structure -->
  <!-- (omitted for brevity - see full example in examples/module-1/) -->

</robot>
```

### Understanding the Structure

Our simple humanoid has:

| Component | Count | Description |
|-----------|-------|-------------|
| Links | 6 | Torso, head, 2 upper arms, 2 lower arms |
| Joints | 5 | Neck, 2 shoulders, 2 elbows |
| Total DOF | 5 | 5 revolute joints = 5 degrees of freedom |

The kinematic tree looks like:

```
base_link (torso)
├── neck_joint ──────────► head
├── left_shoulder_joint ──► left_upper_arm
│   └── left_elbow_joint ──► left_lower_arm
└── right_shoulder_joint ─► right_upper_arm
    └── right_elbow_joint ─► right_lower_arm
```

### Key Design Decisions

**1. Coordinate Frames**: The `origin` element positions each joint relative to its parent. The shoulder joint is at `xyz="0.2 0 0.45"`—meaning 0.2m to the side (X), 0m forward (Y), and 0.45m up (Z) from the torso origin.

**2. Joint Axes**: The shoulder and elbow use `axis xyz="0 1 0"` (Y-axis), creating a forward/back swinging motion. The neck uses `axis xyz="0 0 1"` (Z-axis) for left/right head turning.

**3. Joint Limits**: The elbow has `lower="0"` because human elbows don't bend backward. The shoulder allows full rotation (`-3.14` to `3.14` radians ≈ ±180°).

### Real Humanoids Are More Complex

Our 5-DOF model is simplified for learning. Production humanoid robots have:

| Robot | Approximate DOF |
|-------|-----------------|
| Simple teaching model | 5-6 |
| Boston Dynamics Atlas | 28+ |
| Tesla Optimus | 28+ |
| Honda ASIMO | 57 |
| Full human model | 200+ |

The same URDF principles apply regardless of complexity—just more links, more joints, and more careful coordinate frame management.

## Connecting URDF to ROS 2

A URDF file by itself is just static XML. To bring it alive in ROS 2, we use the **robot_state_publisher** node. This is the bridge between robot structure and the running system.

### How robot_state_publisher Works

The robot_state_publisher performs a crucial function:

1. **Reads** the URDF to understand robot structure
2. **Subscribes** to `/joint_states` topic for current joint positions
3. **Computes** the position of every link based on joint angles
4. **Publishes** coordinate transforms (TF) for the entire robot

```
┌─────────────────┐
│      URDF       │ (static structure)
└────────┬────────┘
         │
         ▼
┌─────────────────┐      /joint_states       ┌─────────────────┐
│ robot_state_    │◄─────────────────────────│ Joint Encoders  │
│ publisher       │      (positions)          │ (hardware/sim)  │
└────────┬────────┘                          └─────────────────┘
         │
         │ /tf (transforms)
         ▼
┌─────────────────┐
│  RViz, Motion   │
│  Planners, etc. │
└─────────────────┘
```

### The TF System

**TF (Transforms)** is ROS 2's system for tracking coordinate frames. Every link in your URDF becomes a coordinate frame, and TF knows how they relate spatially.

When you query "where is the left hand?", TF computes:
1. Base link position
2. Plus shoulder joint rotation
3. Plus elbow joint rotation
4. Equals hand position in world coordinates

This chain of transforms is computed automatically from URDF + joint states.

### Launching robot_state_publisher

To use robot_state_publisher with your URDF:

```bash
# Method 1: Pass URDF file path
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat simple_humanoid.urdf)"

# Method 2: Use a launch file (preferred for complex setups)
ros2 launch my_robot_package robot.launch.py
```

A typical launch file might look like:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.path.dirname(__file__), 'simple_humanoid.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    ])
```

### Visualizing in RViz

Once robot_state_publisher is running, you can visualize your robot:

```bash
# Start RViz
ros2 run rviz2 rviz2

# In RViz:
# 1. Set Fixed Frame to "base_link"
# 2. Add → RobotModel display
# 3. Your robot should appear!
```

To see the robot move, you need to publish joint states:

```bash
# Publish test joint positions
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['neck_joint', 'left_shoulder_joint', 'left_elbow_joint',
         'right_shoulder_joint', 'right_elbow_joint'],
  position: [0.5, 0.3, 1.0, -0.3, 1.0],
  velocity: [],
  effort: []
}"
```

### The Complete Picture

Putting it all together:

```
┌─────────────┐
│    URDF     │──────► robot_state_publisher ──────► /tf
│  (skeleton) │                │
└─────────────┘                │
                               │
┌─────────────┐                ▼
│   Sensors   │──────► /joint_states
│  (encoders) │
└─────────────┘


         ┌─────────────────────────────────────┐
         │           Consumers of /tf          │
         ├─────────────┬─────────────┬─────────┤
         │   RViz      │   Motion    │  Your   │
         │(visualize)  │  Planner    │  Code   │
         └─────────────┴─────────────┴─────────┘
```

This is how the robot's "body map" comes alive—structure (URDF) combined with state (/joint_states) produces spatial awareness (/tf).

:::info Nervous System Complete
With URDF and robot_state_publisher, your robot has proprioception—awareness of its own body position. Combined with the communication patterns from earlier chapters, you now have a complete robotic nervous system foundation.
:::

## Summary

In this chapter, you learned how robots describe their physical structure using URDF:

**Key Concepts:**

- **URDF** is an XML format describing robot structure
  - Defines what parts exist and how they connect
  - Provides geometry for visualization and simulation
  - Includes physical properties for dynamics

- **Links** are rigid body segments (like bones)
  - Visual geometry for display
  - Collision geometry for physics
  - Inertial properties for dynamics

- **Joints** connect links and define motion
  - Types: revolute, continuous, prismatic, fixed
  - Parent/child relationship forms kinematic tree
  - Limits constrain range of motion

- **robot_state_publisher** brings URDF alive
  - Reads static structure from URDF
  - Subscribes to /joint_states for positions
  - Publishes TF transforms for spatial awareness

- **TF** tracks all coordinate frames
  - Every link becomes a frame
  - Transforms chain through joints
  - Enables queries like "where is the hand?"

**Nervous System Parallels:**
- URDF = Anatomical blueprint
- Links = Bones
- Joints = Articulation points
- robot_state_publisher = Proprioceptive system
- TF = Body map in the brain

**Module Complete:**

Congratulations! You've completed Module 1: The Robotic Nervous System. You now understand:

1. **What ROS 2 is** and why middleware matters for robots
2. **How nodes communicate** through topics and services
3. **How to write Python code** using rclpy
4. **How robots describe their bodies** with URDF

This foundation prepares you for everything that follows—perception, planning, control, and learning all build on these fundamentals. The nervous system is ready; now it's time to give your robot intelligence.

---

:::info Module Complete
Congratulations! You have completed Module 1: The Robotic Nervous System. You now understand how ROS 2 provides the communication infrastructure for humanoid robots.
:::
