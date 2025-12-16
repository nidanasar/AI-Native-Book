---
sidebar_position: 2
title: "Chapter 2: Nodes, Topics, and Services"
description: "Master ROS 2 communication patterns: when to use publish/subscribe vs request/response"
---

# Nodes, Topics, and Services

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Explain what a ROS 2 node is and how nodes interact
- Distinguish between topics (publish/subscribe) and services (request/response)
- Identify when to use each communication pattern
- Diagram a simple robot system with nodes, topics, and services
:::

## Understanding Nodes

In Chapter 1, we compared ROS 2 nodes to neurons in the nervous system. Now let's explore exactly what nodes are and how they work.

### What Is a Node?

A **node** is an executable process that performs a specific computation in your robot system. Each node should have a single, well-defined purpose:

- A **camera node** captures and publishes images
- A **motor controller node** receives commands and drives motors
- A **planning node** analyzes sensor data and decides what to do
- A **logger node** records data for later analysis

This single-responsibility principle keeps your code organized and maintainable. If your camera node crashes, your motor controller continues working. If you want to swap cameras, you only modify one node.

### The Node Lifecycle

When a ROS 2 node starts, it goes through several phases:

1. **Initialization**: The node registers itself with the ROS 2 system
2. **Configuration**: Publishers, subscribers, services, and parameters are set up
3. **Active**: The node performs its work, responding to callbacks
4. **Shutdown**: Resources are cleaned up when the node terminates

ROS 2 also supports **managed lifecycle nodes** that provide more control over these transitions—useful for robots that need careful startup and shutdown sequences. For now, we'll focus on standard nodes.

### Node Names and Namespaces

Every node has a unique name within the ROS 2 system. Names follow these conventions:

- Start with a letter or underscore
- Contain only letters, numbers, and underscores
- Are case-sensitive (`CameraNode` differs from `camera_node`)

**Namespaces** group related nodes together and prevent naming conflicts. For example, a robot with two arms might have:

```
/left_arm/joint_controller
/left_arm/gripper_controller
/right_arm/joint_controller
/right_arm/gripper_controller
```

The namespace `/left_arm/` keeps the left arm's nodes separate from the right arm's, even though both have a `joint_controller`.

:::tip Nervous System Parallel
Think of namespaces like the divisions in your nervous system—the nodes controlling your left hand are distinct from those controlling your right hand, even though they perform similar functions.
:::

### Node Discovery

One powerful feature of ROS 2 is **automatic discovery**. When a node starts, it announces its presence. Other nodes can find it without any central server or configuration file. This happens through the underlying DDS middleware.

You can see all active nodes with the command:

```bash
ros2 node list
```

And get details about a specific node:

```bash
ros2 node info /camera_node
```

This shows the node's publishers, subscribers, services, and parameters—everything you need to understand how it connects to the system.

## Topics: Publish/Subscribe Communication

**Topics** are the most common communication mechanism in ROS 2. They implement a **publish/subscribe** pattern where:

- **Publishers** send messages to a named topic
- **Subscribers** receive messages from that topic
- Publishers and subscribers don't know about each other directly

### How Topics Work

Consider a camera streaming images to a vision processing system:

```
┌─────────────┐                          ┌─────────────┐
│ Camera Node │ ──── /camera/image ────▶ │ Vision Node │
│ (publisher) │                          │(subscriber) │
└─────────────┘                          └─────────────┘
```

The camera node doesn't care who receives the images—it just publishes them. The vision node doesn't care which camera produces the images—it just processes what arrives. This **loose coupling** is fundamental to ROS 2's flexibility.

### Many-to-Many Communication

Topics support multiple publishers and multiple subscribers:

```
┌─────────────┐
│ Camera 1    │ ────┐
└─────────────┘     │                    ┌─────────────┐
                    ├── /sensor_data ───▶│ Logger Node │
┌─────────────┐     │                    └─────────────┘
│ Camera 2    │ ────┤                          ▲
└─────────────┘     │                          │
                    │                    ┌─────────────┐
┌─────────────┐     │                    │ Display Node│
│ IMU Sensor  │ ────┘                    └─────────────┘
└─────────────┘
```

All three sensors publish to `/sensor_data`, and both the logger and display receive all messages. This pattern is common for:

- Logging systems that record everything
- Monitoring dashboards
- Multi-sensor fusion algorithms

### Message Types

Every topic has a **message type** that defines the structure of data it carries. ROS 2 provides many standard message types:

| Package | Type | Description |
|---------|------|-------------|
| `std_msgs` | `String` | Simple text string |
| `std_msgs` | `Int32`, `Float64` | Numeric values |
| `geometry_msgs` | `Twist` | Linear and angular velocity |
| `geometry_msgs` | `Pose` | Position and orientation |
| `sensor_msgs` | `Image` | Camera images |
| `sensor_msgs` | `JointState` | Joint positions, velocities, efforts |

When you create a publisher or subscriber, you specify the message type:

```python
from std_msgs.msg import String

# Publisher sends String messages on 'robot_status' topic
publisher = node.create_publisher(String, 'robot_status', 10)

# Subscriber receives String messages from 'robot_status' topic
subscription = node.create_subscription(String, 'robot_status', callback, 10)
```

The publisher and subscriber must use the same message type, or communication fails.

### Quality of Service (QoS)

The `10` in the examples above is a **QoS (Quality of Service)** setting—specifically, the queue depth. But QoS encompasses much more:

| QoS Policy | Options | Use Case |
|------------|---------|----------|
| **Reliability** | Best effort, Reliable | Sensor data (best effort) vs commands (reliable) |
| **Durability** | Volatile, Transient local | Whether late subscribers get old messages |
| **History** | Keep last N, Keep all | How many messages to buffer |
| **Deadline** | Time duration | Maximum time between messages |

For most applications, default QoS settings work well. Advanced tuning is needed for:

- Real-time control systems requiring guaranteed delivery
- High-bandwidth sensors where dropping some data is acceptable
- Distributed systems across unreliable networks

:::warning Common Pitfall
Mismatched QoS settings between publisher and subscriber can cause silent communication failures. If your subscriber isn't receiving messages, check QoS compatibility first.
:::

### Topics in the Nervous System

In our nervous system analogy, topics are like **neural pathways**:

- Your optic nerve carries visual signals from eye to brain (continuous stream)
- Your auditory nerve carries sound information (continuous stream)
- Multiple brain regions can process the same visual input (multiple subscribers)

Topics excel at continuous, streaming data—exactly what sensory systems produce.

## Services: Request/Response Patterns

While topics handle streaming data, **services** handle discrete operations that need a response. Services implement a **request/response** pattern:

- A **client** sends a request and waits for a response
- A **server** processes the request and returns a response
- Communication is synchronous (client blocks until response arrives)

### How Services Work

Consider a robot that needs to calibrate a sensor:

```
┌─────────────┐     Request: "calibrate"    ┌─────────────┐
│ Main Node   │ ─────────────────────────▶  │ Sensor Node │
│  (client)   │                             │  (server)   │
│             │ ◀────────────────────────── │             │
└─────────────┘     Response: "success"     └─────────────┘
```

The main node asks the sensor to calibrate and waits for confirmation. This is fundamentally different from topics:

- **Blocking**: The client waits for the response
- **Guaranteed delivery**: You know the server received the request
- **One-to-one**: Each request gets exactly one response

### Service Definitions

Services are defined by a **service type** that specifies the request and response structure:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

The `---` separates request fields (above) from response fields (below). Built-in service types include:

| Package | Type | Use Case |
|---------|------|----------|
| `std_srvs` | `Empty` | Trigger with no data (e.g., "reset") |
| `std_srvs` | `SetBool` | Enable/disable something |
| `std_srvs` | `Trigger` | Trigger action, get success/message |
| `example_interfaces` | `AddTwoInts` | Simple calculation example |

### Server Implementation

A service server advertises a service and handles incoming requests:

```python
from example_interfaces.srv import AddTwoInts

def add_callback(request, response):
    response.sum = request.a + request.b
    return response

service = node.create_service(AddTwoInts, 'add_two_ints', add_callback)
```

The callback function:
1. Receives the request object with `a` and `b` fields
2. Fills in the response object with `sum`
3. Returns the response

### Client Implementation

A service client sends requests and receives responses:

```python
from example_interfaces.srv import AddTwoInts

client = node.create_client(AddTwoInts, 'add_two_ints')

# Wait for service to be available
while not client.wait_for_service(timeout_sec=1.0):
    print('Waiting for service...')

# Create and send request
request = AddTwoInts.Request()
request.a = 5
request.b = 3

# Call service (async)
future = client.call_async(request)

# Wait for response
rclpy.spin_until_future_complete(node, future)
print(f'Result: {future.result().sum}')  # Output: 8
```

The client must:
1. Wait for the service to become available
2. Create a request object with the required fields
3. Send the request and wait for the response

### Services in the Nervous System

In our nervous system analogy, services are like **reflexes**:

- Tap your knee (request) → leg kicks (response)
- Touch something hot (request) → pull hand back (response)

Reflexes are:
- Triggered on demand (not continuous)
- Immediate response expected
- Specific stimulus produces specific response

This maps perfectly to ROS 2 services: discrete requests that need guaranteed responses.

## When to Use Each Pattern

One of the most important skills in ROS 2 development is choosing the right communication pattern. Here's a decision framework:

### Use Topics When:

| Scenario | Example |
|----------|---------|
| Data flows continuously | Camera frames at 30 FPS |
| Multiple consumers need the same data | Logging, display, and processing all need sensor data |
| Publisher doesn't care who receives | Sensor publishes regardless of subscribers |
| Timing is flexible | Missing one frame isn't critical |
| Data is "fire and forget" | Robot doesn't need to know data was received |

**Real Examples:**
- `/camera/image`: Continuous video stream
- `/joint_states`: Current positions of all joints
- `/cmd_vel`: Velocity commands to drive base
- `/imu/data`: Accelerometer/gyroscope readings

### Use Services When:

| Scenario | Example |
|----------|---------|
| Operation needs confirmation | "Did calibration succeed?" |
| Operation happens occasionally | Configuration changes, mode switches |
| Client needs to wait for result | Query for current state |
| Request-response is natural | "What's your status?" → "Status: OK" |
| Operation must not be missed | Critical commands |

**Real Examples:**
- `/calibrate_sensor`: Trigger calibration, confirm success
- `/set_mode`: Switch between walking and standing
- `/get_battery_level`: Query current charge
- `/emergency_stop`: Critical halt command

### Use Actions When (Preview):

For completeness, **actions** combine aspects of both patterns:

| Scenario | Example |
|----------|---------|
| Long-running tasks | Navigation to a goal |
| Progress feedback needed | "50% complete..." |
| Cancellation support | "Stop moving, obstacle detected!" |
| Asynchronous completion | Task runs while robot continues operating |

We'll cover actions in detail in a later module.

### Decision Flowchart

```
Is the data continuous/streaming?
├─ Yes → Use TOPIC
└─ No → Do you need a response?
         ├─ No → Use TOPIC (one-shot publish)
         └─ Yes → Will it take a long time?
                  ├─ No → Use SERVICE
                  └─ Yes → Use ACTION
```

### Mixed Patterns in Practice

Real robots use all patterns together. Here's a humanoid robot example:

```
TOPICS (continuous data):
├── /camera/image           (30 FPS video)
├── /joint_states           (100 Hz joint positions)
├── /cmd_vel               (velocity commands)
└── /imu/data              (200 Hz IMU readings)

SERVICES (discrete operations):
├── /calibrate_joints       (run calibration routine)
├── /set_control_mode       (position vs velocity control)
├── /get_diagnostics        (query system health)
└── /emergency_stop         (halt all motion)

ACTIONS (long-running tasks):
├── /navigate_to_pose       (walk to location)
├── /pick_object           (grasp sequence)
└── /stand_up              (rising from seated)
```

:::info Design Pattern
Start with topics for data flow and services for commands. Add actions when you find yourself polling topics to check if a task is complete.
:::

## Putting It Together: A Complete Example

Let's diagram a simple humanoid upper body system that combines everything we've learned:

```
                                    ┌─────────────────────┐
                                    │    Vision Node      │
                    /camera/image   │  - Detects objects  │
┌──────────────┐ ─────────────────▶ │  - Publishes poses  │
│ Camera Node  │                    └─────────┬───────────┘
└──────────────┘                              │
                                              │ /detected_objects
                                              ▼
                                    ┌─────────────────────┐
                                    │   Planning Node     │
/joint_states                       │  - Decides actions  │
┌──────────────┐ ─────────────────▶ │  - Plans motions    │
│ Joint State  │                    └─────────┬───────────┘
│  Publisher   │                              │
└──────────────┘                              │ /arm_commands
                                              ▼
                                    ┌─────────────────────┐
        /calibrate_arm [SERVICE]    │   Motor Controller  │
┌──────────────┐ ═════════════════▶ │  - Executes motion  │
│ Operator     │                    │  - Reports status   │
│  Interface   │ ◀═════════════════ │                     │
└──────────────┘    [Response]      └─────────────────────┘
```

In this system:

**Topics (solid arrows):**
- Camera publishes images continuously
- Vision node publishes detected object positions
- Joint state publisher reports current positions
- Planning node publishes arm movement commands

**Services (double arrows):**
- Operator interface can trigger arm calibration
- Motor controller confirms calibration success

Each node has a single responsibility, communicates through well-defined interfaces, and can be developed and tested independently.

## Summary

In this chapter, you learned the core communication patterns in ROS 2:

**Key Concepts:**

- **Nodes** are independent processes with single, well-defined purposes
  - Have unique names within namespaces
  - Discovered automatically by the ROS 2 system
  - Can fail independently without crashing the whole robot

- **Topics** implement publish/subscribe communication
  - Ideal for continuous, streaming data
  - Support many-to-many connections
  - Publishers and subscribers are loosely coupled
  - Use message types to define data structure

- **Services** implement request/response communication
  - Ideal for discrete, synchronous operations
  - Client waits for server response
  - Guaranteed delivery of request/response pairs
  - Use service types defining request and response

- **Pattern Selection Guidelines:**
  - Streaming data → Topics
  - Need confirmation → Services
  - Long-running tasks → Actions (covered later)

**Nervous System Parallels:**
- Nodes = Neurons (specialized processors)
- Topics = Neural pathways (continuous streams)
- Services = Reflexes (stimulus → response)
- Namespaces = Nervous system divisions (left arm vs right arm)

**What's Next:**

Now that you understand how ROS 2 nodes communicate, you're ready to write actual code. In Chapter 3, we'll use Python and rclpy to create publishers, subscribers, and services—turning theory into working programs.

---

:::tip Next Chapter
Continue to [Chapter 3: Bridging Python Agents to ROS 2 Controllers](./python-rclpy-bridge) to write your first ROS 2 Python code.
:::
