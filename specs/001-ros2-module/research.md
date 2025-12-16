# Research: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-13
**Branch**: `001-ros2-module`
**Status**: Complete

## Research Objectives

1. Verify ROS 2 Humble/Iron API stability and compatibility
2. Document accurate ROS 2 terminology and architecture
3. Identify appropriate code examples from official sources
4. Research URDF best practices for educational content
5. Catalog common learner misconceptions to address

---

## 1. ROS 2 Version Selection

### Decision: Target ROS 2 Humble and Iron

**Rationale**:
- Humble Hawksbill (May 2022): LTS release, supported until May 2027
- Iron Irwini (May 2023): Latest stable, supported until November 2024
- Both use same core APIs; code is compatible between versions

**Alternatives Considered**:
- Rolling: Too unstable for educational content
- Foxy: End of life (May 2023)

**Source**: ROS 2 Distributions (https://docs.ros.org/en/humble/Releases.html)

---

## 2. Core Concepts Verification

### 2.1 Nodes

**Definition**: A node is a process that performs computation. Nodes communicate with each other via topics, services, and actions.

**Key Points for Chapter 2**:
- Each node should have a single, well-defined purpose
- Nodes are identified by name (must be unique in the graph)
- Lifecycle nodes provide managed state transitions
- Nodes can be composed into a single process for efficiency

**Nervous System Analogy**: Nodes are like specialized neurons—each performs a specific function but contributes to the overall system.

**Source**: https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html

### 2.2 Topics

**Definition**: Topics are named buses over which nodes exchange messages. Topics implement a publish/subscribe model.

**Key Points for Chapter 2**:
- Anonymous publish/subscribe (publishers don't know subscribers)
- Many-to-many communication pattern
- Quality of Service (QoS) settings control reliability, durability, etc.
- Message types are strongly typed (e.g., std_msgs/String)

**Common Message Types**:
- `std_msgs/String`: Simple string data
- `std_msgs/Int32`: Integer data
- `geometry_msgs/Twist`: Velocity commands (linear + angular)
- `sensor_msgs/Image`: Camera images

**Nervous System Analogy**: Topics are like nerve pathways—continuous streams of sensory or motor signals.

**Source**: https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html

### 2.3 Services

**Definition**: Services implement a request/response communication pattern. A client sends a request, and a server sends a response.

**Key Points for Chapter 2**:
- Synchronous communication (client waits for response)
- One server, multiple clients allowed
- Defined by service type (request + response message)
- Used for discrete operations (not streaming data)

**When to Use Topics vs Services**:

| Use Case | Pattern | Example |
|----------|---------|---------|
| Continuous sensor data | Topic | Camera images, joint states |
| Periodic commands | Topic | Velocity commands |
| One-time configuration | Service | Set parameters, calibrate |
| State queries | Service | Get robot status |
| Long-running tasks | Action | Navigation goals |

**Nervous System Analogy**: Services are like reflexes—a stimulus triggers a specific, immediate response.

**Source**: https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html

### 2.4 Actions

**Definition**: Actions are for long-running tasks. They provide feedback during execution and can be canceled.

**Key Points for Chapter 2** (mention briefly, detail in later module):
- Asynchronous with feedback
- Built on topics and services internally
- Preemptable (can be canceled mid-execution)
- Used for navigation, manipulation, etc.

**Source**: https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html

---

## 3. Python Client Library (rclpy)

### 3.1 API Verification

**Verified APIs for Chapter 3**:

```python
# Node creation
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('node_name')
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

```python
# Publisher
from std_msgs.msg import String
publisher = node.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello'
publisher.publish(msg)
```

```python
# Subscriber
def callback(msg):
    print(msg.data)
subscription = node.create_subscription(String, 'topic_name', callback, 10)
```

```python
# Service Server
from example_interfaces.srv import AddTwoInts
def callback(request, response):
    response.sum = request.a + request.b
    return response
service = node.create_service(AddTwoInts, 'add_two_ints', callback)
```

```python
# Service Client
client = node.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 1
request.b = 2
future = client.call_async(request)
```

**Source**: https://docs.ros2.org/latest/api/rclpy/

### 3.2 Common Patterns

**Minimal Publisher Node** (official example):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

**Source**: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

---

## 4. URDF Research

### 4.1 URDF Structure

**Definition**: Unified Robot Description Format (URDF) is an XML format for describing robot models.

**Core Elements**:

| Element | Description |
|---------|-------------|
| `<robot>` | Root element, contains name attribute |
| `<link>` | Rigid body with visual, collision, inertial properties |
| `<joint>` | Connection between links with type and limits |
| `<material>` | Visual appearance (color, texture) |

**Joint Types**:
- `revolute`: Rotates around axis with limits
- `continuous`: Rotates around axis without limits
- `prismatic`: Slides along axis with limits
- `fixed`: No movement (rigid connection)
- `floating`: 6-DOF joint (rare)
- `planar`: 2-DOF motion in a plane

### 4.2 Simplified Humanoid Design

**Proposed Structure** (6 joints, 7 links):

```
base_link (torso)
├── head_joint (fixed) → head_link
├── left_shoulder_joint (revolute) → left_upper_arm_link
│   └── left_elbow_joint (revolute) → left_lower_arm_link
└── right_shoulder_joint (revolute) → right_upper_arm_link
    └── right_elbow_joint (revolute) → right_lower_arm_link
```

**Rationale**:
- Demonstrates kinematic chain concept
- Shows parent-child relationships
- Includes both fixed and revolute joints
- Traceable by learners in under 5 minutes
- Can be visualized in RViz/Gazebo

### 4.3 robot_state_publisher

**Purpose**: Publishes the state of a robot to tf2 based on URDF and joint states.

**Integration Point**:
- Reads URDF from `robot_description` parameter
- Subscribes to `/joint_states` topic
- Publishes transforms to `/tf` and `/tf_static`

**Key for Chapter 4**: Demonstrates how URDF connects to the ROS 2 "nervous system"—the static robot description combined with dynamic joint states produces real-time spatial awareness.

**Source**: https://github.com/ros/robot_state_publisher

---

## 5. Common Misconceptions

### Misconceptions to Address

| Misconception | Correction | Where to Address |
|---------------|------------|------------------|
| "ROS is an operating system" | ROS is middleware; runs ON an OS | Chapter 1 |
| "Topics are like function calls" | Topics are async streams; use services for request/response | Chapter 2 |
| "One node per sensor/actuator" | Nodes group related functionality; not 1:1 mapping | Chapter 2 |
| "URDF defines robot behavior" | URDF is static description; behavior comes from code | Chapter 4 |
| "ROS 2 requires Linux" | ROS 2 supports Linux, macOS, Windows | Chapter 1 |

---

## 6. Reference Materials

### Official Documentation

- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- rclpy API Reference: https://docs.ros2.org/latest/api/rclpy/
- URDF Specification: http://wiki.ros.org/urdf/XML
- tf2 Documentation: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html

### Tutorials (for code validation)

- Writing a Publisher/Subscriber (Python): https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- Writing a Service/Client (Python): https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
- Building a Visual Robot Model (URDF): https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html

### Academic References

- Quigley, M., et al. (2009). ROS: an open-source Robot Operating System. ICRA Workshop on Open Source Software.
- Macenski, S., et al. (2022). Robot Operating System 2: Design, Architecture, and Uses in the Wild. Science Robotics.

---

## 7. Research Conclusions

All technical questions resolved:

| Question | Resolution |
|----------|------------|
| Which ROS 2 version? | Humble/Iron (LTS) |
| Which APIs to use? | Direct rclpy, no wrappers |
| URDF complexity? | Simplified 6-joint humanoid |
| Code source? | Adapted from official tutorials |
| Message types? | std_msgs, geometry_msgs |

**Ready for Phase 1**: Content model and quickstart guide.
