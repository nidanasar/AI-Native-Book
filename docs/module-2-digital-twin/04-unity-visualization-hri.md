---
sidebar_position: 4
title: "Chapter 4: High-Fidelity Visualization and Interaction with Unity"
description: "Use Unity for realistic rendering and human-robot interaction scenarios"
---

# High-Fidelity Visualization and Interaction with Unity

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Explain when to use Unity vs Gazebo for robotics simulation
- Understand the ROS-TCP-Connector architecture for Unity-ROS 2 communication
- Describe how to visualize robot state in Unity from ROS 2 data
- Appreciate Unity's role in human-robot interaction (HRI) research
:::

## Gazebo vs Unity: When to Use Each

Throughout this module, you've used Gazebo for physics simulation. But Gazebo isn't the only tool in the roboticist's toolkit. **Unity**, a popular game engine, offers capabilities that complement Gazebo's strengths. Understanding when to use each tool—and how to use them together—is essential for modern robotics development.

### Comparing the Two Approaches

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Primary Strength** | Physics accuracy | Visual fidelity |
| **Physics Engine** | DART, ODE, Bullet (scientific) | PhysX (game-oriented) |
| **Rendering** | Functional | Photorealistic |
| **ROS Integration** | Native (ros_gz_bridge) | Via ROS-TCP-Connector |
| **Learning Curve** | Moderate | Higher (full game engine) |
| **Best For** | Algorithm development | Visualization, HRI, training data |

### When to Choose Gazebo

Use Gazebo when your priority is **physics accuracy**:

- **Control algorithm development**: Walking, balancing, manipulation
- **Sensor simulation**: LiDAR, depth cameras, IMU with configurable noise
- **Rapid prototyping**: Quick iteration on robot behaviors
- **Research validation**: Reproducible physics for scientific claims

Gazebo's physics engines (especially DART) are designed for robotics, with accurate contact modeling, constraint satisfaction, and real-time performance.

### When to Choose Unity

Use Unity when your priority is **visual realism or human interaction**:

- **Vision system training**: Photorealistic images for computer vision
- **Human-robot interaction**: Scenarios with realistic virtual humans
- **Demonstrations**: Impressive visualizations for stakeholders
- **VR/AR integration**: Immersive robot interfaces
- **Synthetic data generation**: Large-scale image datasets

Unity's rendering pipeline can produce images indistinguishable from photographs—something Gazebo cannot match.

### The Complementary Approach

Many robotics teams use **both** tools:

```
┌─────────────────────────────────────────────────────────────────┐
│                    COMPLEMENTARY WORKFLOW                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Development Phase        →    Visualization Phase              │
│   ┌─────────────┐              ┌─────────────┐                  │
│   │   Gazebo    │              │    Unity    │                  │
│   │  (physics)  │────ROS 2────►│  (visuals)  │                  │
│   └─────────────┘              └─────────────┘                  │
│                                                                  │
│   - Test algorithms            - Create demos                    │
│   - Validate control           - Train vision                    │
│   - Generate sensor data       - Study HRI                       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

Your robot code runs in ROS 2, subscribes to Gazebo's physics simulation, and simultaneously visualizes in Unity. The same `/joint_states` topic drives both the simulated robot and the Unity visualization.

:::tip Key Insight
Gazebo and Unity aren't competitors—they're complementary. Use Gazebo to make your robot work correctly, and Unity to make it look impressive and interact with humans.
:::

## Introduction to Unity for Robotics

Unity is a game engine used by millions of developers worldwide. In recent years, it has gained significant traction in robotics research, particularly for simulation and synthetic data generation.

### Why Game Engines for Robotics?

Game engines solve problems that overlap with robotics:

| Game Development Need | Robotics Application |
|----------------------|---------------------|
| Realistic 3D worlds | Robot operating environments |
| Character animation | Human models for HRI |
| Physics simulation | Object interactions (though less accurate) |
| Real-time rendering | Sensor visualization |
| VR/AR support | Immersive robot interfaces |

Unity's mature ecosystem provides tools that would take years to build from scratch.

### Unity Robotics Hub

Unity provides official robotics support through the **Unity Robotics Hub**, which includes:

- **URDF Importer**: Load your robot models directly from URDF files
- **ROS-TCP-Connector**: Bridge between Unity and ROS/ROS 2
- **Sensor packages**: Simulated cameras, LiDAR (basic)
- **Example projects**: Pre-built robotics demonstrations

The Robotics Hub makes Unity accessible to roboticists who aren't game developers.

### URDF Importer

The URDF Importer allows you to bring your robot directly into Unity:

1. Export your URDF (the same file used in Gazebo)
2. Import into Unity via the URDF Importer package
3. Unity creates GameObjects for each link
4. Joints are configured automatically with ArticulationBody components

```
URDF File                    Unity Scene
───────────────────────────────────────────────────
<robot name="humanoid">     Humanoid (GameObject)
  <link name="base">   →      ├── base (ArticulationBody)
  <link name="arm">    →      │   └── arm (ArticulationBody)
  <joint name="...">   →      └── [Joint configured automatically]
</robot>
```

This means your Module 1 URDF can be used in Unity with minimal modification.

### Basic Scene Setup

A typical Unity robotics scene contains:

```
Unity Scene Hierarchy
├── Main Camera
├── Directional Light
├── Ground Plane (with collision)
├── Robot (imported from URDF)
│   ├── base_link
│   ├── torso
│   └── ... (kinematic chain)
├── ROS Connection (script component)
└── Environment Objects
```

Unity uses a left-handed coordinate system (Y-up) while ROS uses right-handed (Z-up). The URDF Importer handles this conversion automatically.

## Connecting Unity to ROS 2

Unity and ROS 2 speak different languages. The **ROS-TCP-Connector** package provides the translation layer, enabling Unity to publish and subscribe to ROS 2 topics.

### ROS-TCP-Connector Architecture

The connection works through a TCP socket:

```
┌─────────────────────────────────────────────────────────────────┐
│                     CONNECTION ARCHITECTURE                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Unity Application              ROS 2 System                    │
│   ┌──────────────────┐          ┌──────────────────┐            │
│   │  Unity Scene     │          │   ROS 2 Nodes    │            │
│   │  ┌────────────┐  │          │  ┌────────────┐  │            │
│   │  │ ROS        │  │   TCP    │  │ ros_tcp_   │  │            │
│   │  │ Connection │◄─┼──────────┼─►│ endpoint   │  │            │
│   │  │ Script     │  │  :10000  │  │   Node     │  │            │
│   │  └────────────┘  │          │  └────────────┘  │            │
│   └──────────────────┘          └──────────────────┘            │
│                                                                  │
│   Messages serialized           Messages deserialized            │
│   to ROS format                 and published to topics          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

Key components:

1. **Unity side**: ROS-TCP-Connector package (C# scripts)
2. **ROS 2 side**: `ros_tcp_endpoint` node (Python)
3. **Communication**: TCP socket on port 10000 (default)

### Setting Up the Connection

**On the ROS 2 side:**

```bash
# Install the endpoint package
sudo apt install ros-humble-ros-tcp-endpoint

# Or from source
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ..
colcon build
source install/setup.bash

# Launch the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**On the Unity side:**

1. Install the ROS-TCP-Connector via Package Manager
2. Add `ROSConnection` component to a GameObject
3. Configure the ROS IP address and port
4. Use `ROSConnection.Instance` to publish/subscribe

### Message Types and Serialization

The ROS-TCP-Connector supports standard ROS message types:

| ROS Message Type | Unity Equivalent |
|-----------------|------------------|
| `sensor_msgs/JointState` | `JointStateMsg` |
| `geometry_msgs/Pose` | `PoseMsg` |
| `std_msgs/Float64` | `Float64Msg` |
| `sensor_msgs/Image` | `ImageMsg` |

Messages are serialized to ROS binary format, ensuring compatibility with existing ROS tools.

### Example: Subscribing to Joint States

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    void Start()
    {
        // Subscribe to /joint_states topic
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
            "/joint_states",
            OnJointStateReceived
        );
    }

    void OnJointStateReceived(JointStateMsg msg)
    {
        // Process joint positions
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];
            float position = (float)msg.position[i];

            Debug.Log($"Joint {jointName}: {position} rad");
            // Update robot visualization...
        }
    }
}
```

## Visualizing Robot State

With the ROS connection established, you can drive your Unity robot visualization from ROS 2 data.

### Joint State Visualization

The most common visualization task is showing joint positions:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Sensor;

public class RobotVisualizer : MonoBehaviour
{
    // Maps joint names to Unity ArticulationBody components
    private Dictionary<string, ArticulationBody> jointMap;

    void Start()
    {
        // Build joint map from imported URDF
        jointMap = new Dictionary<string, ArticulationBody>();
        foreach (var joint in GetComponentsInChildren<ArticulationBody>())
        {
            jointMap[joint.name] = joint;
        }

        // Subscribe to joint states
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
            "/joint_states",
            UpdateVisualization
        );
    }

    void UpdateVisualization(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            if (jointMap.TryGetValue(msg.name[i], out var joint))
            {
                // Set joint target position
                var drive = joint.xDrive;
                drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
                joint.xDrive = drive;
            }
        }
    }
}
```

### TF Visualization

The TF (Transform) system tracks coordinate frames. Unity can visualize these:

```csharp
// Subscribe to TF and visualize coordinate frames
ROSConnection.GetOrCreateInstance().Subscribe<TFMessageMsg>(
    "/tf",
    OnTFReceived
);

void OnTFReceived(TFMessageMsg msg)
{
    foreach (var transform in msg.transforms)
    {
        // Update Unity Transform based on TF data
        string frameName = transform.child_frame_id;
        var position = transform.transform.translation;
        var rotation = transform.transform.rotation;

        // Apply to corresponding Unity GameObject
        UpdateFrame(frameName, position, rotation);
    }
}
```

### Real-Time Updates

For smooth visualization, consider:

- **Update rate matching**: Match Unity's frame rate to ROS publish rate
- **Interpolation**: Smooth between received positions
- **Latency hiding**: Predict positions based on velocity

```csharp
// Simple interpolation for smoother visualization
void Update()
{
    foreach (var joint in joints)
    {
        joint.currentPosition = Mathf.Lerp(
            joint.currentPosition,
            joint.targetPosition,
            Time.deltaTime * smoothingFactor
        );
    }
}
```

## Human-Robot Interaction Scenarios

Unity's greatest strength for robotics lies in **Human-Robot Interaction (HRI)** research. While Gazebo excels at robot physics, Unity excels at simulating the human side of the equation.

### Why Unity for HRI?

HRI research requires:

| Requirement | Unity Capability |
|-------------|-----------------|
| Realistic humans | Character models, animations |
| Natural environments | Photorealistic rendering |
| User studies | VR/AR integration |
| Varied scenarios | Easy scene modification |
| Data collection | Built-in analytics |

Creating these in Gazebo would require enormous custom development.

### Virtual Human Models

Unity provides access to:

- **Character assets**: Realistic 3D human models
- **Animation systems**: Walking, gesturing, facial expressions
- **Behavior trees**: AI-driven human behaviors
- **Crowd simulation**: Multiple humans in a scene

Example HRI scenarios:

```
┌─────────────────────────────────────────────────────────────────┐
│                      HRI SCENARIOS IN UNITY                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   1. Social Navigation                                           │
│      Robot navigates around walking humans                       │
│      Tests: path planning with dynamic obstacles                 │
│                                                                  │
│   2. Handover Tasks                                              │
│      Robot hands object to human                                 │
│      Tests: approach angle, timing, safety                       │
│                                                                  │
│   3. Gesture Recognition                                         │
│      Human gestures, robot responds                              │
│      Tests: perception, interpretation, response                 │
│                                                                  │
│   4. Personal Space                                              │
│      Robot maintains appropriate distance                        │
│      Tests: proxemics, cultural considerations                   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### VR/AR Integration

Unity's VR support enables immersive HRI research:

- **First-person robot control**: Experience from the robot's perspective
- **Telepresence**: Control a robot through VR interface
- **User studies**: Test human comfort with robot behaviors
- **Training**: Teach operators to work with robots

```
VR Headset ──► Unity ──► ROS 2 ──► Robot (simulated or real)
     │                      │
     └──── User Input ──────┘
```

This pipeline allows a human wearing a VR headset to interact with a robot in real-time, with Unity providing the visual interface and ROS 2 handling robot control.

### Synthetic Data for Vision

Unity can generate unlimited training data for vision systems:

- **Domain randomization**: Vary lighting, textures, backgrounds
- **Perfect labels**: Automatic ground truth (bounding boxes, segmentation)
- **Edge cases**: Simulate rare scenarios (unusual poses, occlusions)
- **Scale**: Generate millions of images without manual labeling

```csharp
// Example: Generate training image with labels
void CaptureTrainingData()
{
    // Randomize scene
    RandomizeLighting();
    RandomizeHumanPose();
    RandomizeBackground();

    // Capture image
    var image = camera.Capture();

    // Generate automatic labels
    var boundingBoxes = GetBoundingBoxes();
    var segmentationMask = GetSegmentationMask();

    // Save for training
    SaveTrainingExample(image, boundingBoxes, segmentationMask);
}
```

This synthetic data can train perception systems that transfer to real-world scenarios.

## Summary

In this chapter, you learned how Unity complements Gazebo for humanoid robotics:

**Key Concepts:**

- **Gazebo vs Unity**: Different tools for different needs
  - Gazebo: Physics accuracy, algorithm development
  - Unity: Visual fidelity, HRI research, synthetic data
  - Often used together via ROS 2

- **ROS-TCP-Connector** bridges Unity and ROS 2
  - TCP socket communication
  - Standard ROS message support
  - Bidirectional data flow

- **Robot visualization** in Unity
  - URDF Importer loads robot models
  - Subscribe to /joint_states for real-time updates
  - TF visualization shows coordinate frames

- **HRI applications** leverage Unity's strengths
  - Realistic human models and animations
  - VR/AR integration for immersive research
  - Synthetic data generation for vision training

**The Complete Digital Twin Stack:**

```
┌─────────────────────────────────────────────────────────────────┐
│                  MODULE 2 COMPLETE STACK                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Unity (Visualization)          Gazebo (Physics)                │
│   ┌────────────────────┐        ┌────────────────────┐          │
│   │ • Photorealistic   │        │ • Accurate physics │          │
│   │ • HRI scenarios    │        │ • Sensor simulation│          │
│   │ • VR/AR support    │        │ • Control testing  │          │
│   └─────────┬──────────┘        └─────────┬──────────┘          │
│             │                              │                     │
│             └──────────┬───────────────────┘                     │
│                        │                                         │
│                  ┌─────┴─────┐                                   │
│                  │   ROS 2   │                                   │
│                  │ Middleware│                                   │
│                  └─────┬─────┘                                   │
│                        │                                         │
│                 Your Robot Code                                  │
│                 (same for sim and real!)                         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

**Module 2 Complete:**

Congratulations! You've completed Module 2: The Digital Twin. You now understand:

1. **What digital twins are** and why simulation matters
2. **How to create physics simulations** with Gazebo
3. **How to simulate sensors** for perception development
4. **How to visualize and interact** using Unity

Your humanoid robot now has:
- A **nervous system** (ROS 2 from Module 1)
- A **body** (URDF from Module 1)
- An **imagination** (Gazebo/Unity digital twin from Module 2)

The robot can rehearse movements, test perception algorithms, and interact with virtual humans—all before touching real hardware. This simulation-first approach is how modern humanoid robots are developed.

---

:::info Module Complete
Congratulations! You have completed Module 2: The Digital Twin. You now understand how to simulate humanoid robots using Gazebo for physics and Unity for visualization. Your robot's "imagination" is ready—allowing safe, rapid development before real-world deployment.
:::
