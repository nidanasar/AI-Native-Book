---
sidebar_position: 3
title: "Chapter 3: Bridging Python Agents to ROS 2 Controllers"
description: "Write Python code using rclpy to control robot nodes"
---

# Bridging Python Agents to ROS 2 Controllers

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Set up a Python environment for ROS 2 development with rclpy
- Create a basic ROS 2 node in Python
- Publish and subscribe to topics using rclpy
- Call and provide services in Python
:::

## Introduction to rclpy

In the previous chapters, you learned how ROS 2 organizes robot systems into nodes that communicate through topics and services. Now it's time to write actual code. **rclpy** (ROS Client Library for Python) is your gateway to creating ROS 2 applications in Python.

### What is rclpy?

rclpy is the official Python client library for ROS 2. It provides Python bindings to the underlying ROS 2 core libraries, letting you:

- Create and manage nodes
- Publish and subscribe to topics
- Provide and call services
- Work with parameters and timers
- Handle the node lifecycle

Think of rclpy as the "nerve endings" that connect your Python code to the ROS 2 nervous system. Just as neurons have specialized receptors and transmitters, rclpy provides the interfaces your code needs to send and receive signals.

### Why Python for ROS 2?

ROS 2 supports multiple programming languages, with C++ and Python being the most common. Python offers several advantages for learning and rapid development:

| Advantage | Description |
|-----------|-------------|
| **Readability** | Python's clean syntax makes code easier to understand |
| **Rapid prototyping** | No compilation step means faster iteration |
| **Rich ecosystem** | Easy integration with NumPy, OpenCV, PyTorch, etc. |
| **Debugging** | Interactive development and easier troubleshooting |

For production robots requiring maximum performance, C++ (rclcpp) is often preferred. However, many successful robots use Python for high-level logic while delegating time-critical operations to lower-level components. This hybrid approach is common in humanoid robotics.

### The rclpy Architecture

Every rclpy program follows a consistent pattern:

```python
import rclpy
from rclpy.node import Node

# 1. Initialize the ROS 2 Python library
rclpy.init()

# 2. Create your node(s)
node = Node('my_node_name')

# 3. Do work (spin, process callbacks)
rclpy.spin(node)

# 4. Clean up
node.destroy_node()
rclpy.shutdown()
```

The `rclpy.init()` call is essential—it connects your Python process to the ROS 2 middleware. Without it, nothing works. The `rclpy.spin()` function keeps your node alive, processing incoming messages and timer callbacks. When you're done, proper cleanup with `destroy_node()` and `shutdown()` releases resources cleanly.

:::tip Nervous System Parallel
`rclpy.init()` is like waking up—it activates your connection to the nervous system. `rclpy.spin()` is like staying conscious—continuously processing sensory input and generating responses.
:::

## Creating Your First Node

Let's build a simple ROS 2 node step by step. Understanding this foundation will make everything else easier.

### The Node Class

In rclpy, you typically create nodes by subclassing `Node`. This object-oriented approach keeps your code organized and provides access to all ROS 2 functionality:

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        # Initialize with a unique node name
        super().__init__('my_first_node')

        # Log a message to confirm we're running
        self.get_logger().info('Node has started!')
```

The `super().__init__('my_first_node')` call registers your node with ROS 2 using the name `my_first_node`. This name must be unique within your robot system—just like each neuron has a distinct identity within the nervous system.

### Essential Node Methods

The `Node` base class provides many useful methods:

| Method | Purpose |
|--------|---------|
| `get_logger()` | Access the node's logger for output |
| `create_publisher()` | Create a topic publisher |
| `create_subscription()` | Create a topic subscriber |
| `create_service()` | Create a service server |
| `create_client()` | Create a service client |
| `create_timer()` | Create a periodic timer |
| `get_clock()` | Access the node's clock |
| `get_name()` | Get the node's name |

### Logging

ROS 2 provides a structured logging system that's much better than `print()` statements:

```python
# Different log levels for different situations
self.get_logger().debug('Detailed debugging info')    # For development
self.get_logger().info('Normal operation messages')   # Standard output
self.get_logger().warn('Something unexpected')        # Potential issues
self.get_logger().error('Something went wrong')       # Errors
self.get_logger().fatal('Critical failure')          # System failures
```

These log messages are timestamped, tagged with your node name, and can be filtered by severity level—essential for debugging complex robot systems.

### The Complete Node Template

Here's a reusable template for creating ROS 2 nodes:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node initialized')

        # Add publishers, subscribers, services, timers here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The `try/except/finally` block ensures clean shutdown even when you press Ctrl+C to stop the node. This pattern should become muscle memory.

:::warning Common Mistake
Forgetting to call `rclpy.init()` before creating nodes is a frequent error. You'll get cryptic error messages about uninitialized contexts. Always initialize first!
:::

## Publishing and Subscribing to Topics

Topics are the backbone of ROS 2 communication. Let's implement a publisher and subscriber that work together.

### Creating a Publisher

A publisher sends messages to a topic at regular intervals. Here's our minimal publisher example:

```python
#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example
================================
This node publishes status messages to demonstrate the publish/subscribe pattern.
Think of it as a sensory neuron continuously sending signals.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node.

    Publishes string messages to 'robot_status' topic every 0.5 seconds.
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create a publisher
        # Parameters: message type, topic name, queue size
        self.publisher_ = self.create_publisher(
            String,           # Message type
            'robot_status',   # Topic name
            10                # QoS queue depth
        )

        # Create a timer that fires every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track published messages
        self.count = 0
        self.get_logger().info('MinimalPublisher node initialized')

    def timer_callback(self):
        """Called every 0.5 seconds to publish a message."""
        msg = String()
        msg.data = f'Robot status update #{self.count}: All systems operational'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        minimal_publisher.get_logger().info('Shutting down...')
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key points:**

1. **`create_publisher()`** takes three arguments:
   - Message type (`String` from `std_msgs.msg`)
   - Topic name (`'robot_status'`)
   - Queue size (`10` messages)

2. **`create_timer()`** sets up periodic execution—the callback runs every 0.5 seconds

3. **`publish()`** sends the message to all subscribers

### Creating a Subscriber

A subscriber listens for messages on a topic. Here's the corresponding subscriber:

```python
#!/usr/bin/env python3
"""
Minimal ROS 2 Subscriber Example
=================================
This node receives messages from the 'robot_status' topic.
Think of it as a motor neuron receiving commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal ROS 2 subscriber node.

    Listens to 'robot_status' topic and logs received messages.
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,                    # Message type
            'robot_status',            # Topic name
            self.listener_callback,    # Callback function
            10                         # QoS queue depth
        )

        self.get_logger().info('MinimalSubscriber node initialized')
        self.get_logger().info('Listening for messages on "robot_status" topic...')

    def listener_callback(self, msg):
        """Called whenever a message is received."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Shutting down...')
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key points:**

1. **`create_subscription()`** takes four arguments:
   - Message type (must match publisher)
   - Topic name (must match publisher)
   - Callback function (called for each message)
   - Queue size

2. **Callback function** receives the message as its only argument

3. The subscriber automatically receives messages—no polling required

### Running Publisher and Subscriber Together

To see pub/sub in action, open two terminals:

```bash
# Terminal 1: Start the publisher
python3 minimal_publisher.py

# Terminal 2: Start the subscriber
python3 minimal_subscriber.py
```

You'll see the subscriber receiving messages from the publisher. This decoupled communication is the essence of ROS 2—neither node knows about the other; they only know about the topic.

### Message Types

The `String` message is simple, but ROS 2 provides many message types:

```python
from std_msgs.msg import Int32, Float64, Bool
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import Image, LaserScan, JointState
```

For humanoid robots, `JointState` is particularly important—it contains joint positions, velocities, and efforts for the entire robot.

## Working with Services

Services provide request/response communication for operations that need confirmation. Let's implement a service server and client.

### Creating a Service Server

A service server waits for requests and sends responses:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Service Server Example
=====================================
This node provides a service that performs calculations.
Think of it as a reflex arc waiting for stimuli.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleService(Node):
    """
    A simple ROS 2 service server node.

    Provides 'calculate_joint_position' service that adds two integers.
    """

    def __init__(self):
        super().__init__('simple_service')

        # Create a service
        self.srv = self.create_service(
            AddTwoInts,                    # Service type
            'calculate_joint_position',    # Service name
            self.calculate_callback        # Callback function
        )

        self.get_logger().info('SimpleService node initialized')
        self.get_logger().info('Service "calculate_joint_position" is ready')

    def calculate_callback(self, request, response):
        """
        Handle incoming service requests.

        Args:
            request: Contains 'a' and 'b' integers
            response: Must fill 'sum' and return it
        """
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Request received: {request.a} + {request.b} = {response.sum}'
        )

        # IMPORTANT: Services must return the response
        return response


def main(args=None):
    rclpy.init(args=args)
    simple_service = SimpleService()

    try:
        rclpy.spin(simple_service)
    except KeyboardInterrupt:
        simple_service.get_logger().info('Shutting down...')
    finally:
        simple_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key points:**

1. **`create_service()`** takes service type, name, and callback
2. **Callback signature** is `callback(request, response)`
3. **Must return** the response object—forgetting this is a common bug

### Creating a Service Client

A service client sends requests and waits for responses:

```python
#!/usr/bin/env python3
"""
ROS 2 Service Client Example
==============================
This node calls a service and processes the response.
Think of it as triggering a reflex and observing the result.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClient(Node):
    """
    A ROS 2 service client node.

    Calls 'calculate_joint_position' service and handles responses.
    """

    def __init__(self):
        super().__init__('service_client')

        # Create a client for the service
        self.client = self.create_client(
            AddTwoInts,                  # Service type
            'calculate_joint_position'   # Service name
        )

        # Wait for the service to be available
        self.get_logger().info('Waiting for service to become available...')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')

        self.get_logger().info('Service is available!')

    def send_request(self, a, b):
        """Send a request and wait for the response."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Send request asynchronously
        future = self.client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    service_client = ServiceClient()

    try:
        # Send a test request
        value_a = 10
        value_b = 5

        if len(sys.argv) >= 3:
            value_a = int(sys.argv[1])
            value_b = int(sys.argv[2])

        response = service_client.send_request(value_a, value_b)

        service_client.get_logger().info(
            f'Result: {value_a} + {value_b} = {response.sum}'
        )

    except KeyboardInterrupt:
        service_client.get_logger().info('Interrupted by user')
    finally:
        service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key points:**

1. **`wait_for_service()`** blocks until the server is ready—essential for reliability
2. **`call_async()`** sends the request and returns a Future
3. **`spin_until_future_complete()`** blocks until the response arrives

### Running Server and Client Together

```bash
# Terminal 1: Start the service server
python3 simple_service.py

# Terminal 2: Call the service
python3 service_client.py 15 7
# Output: Result: 15 + 7 = 22
```

You can also test services from the command line:

```bash
ros2 service call /calculate_joint_position example_interfaces/srv/AddTwoInts "{a: 10, b: 5}"
```

## Complete Working Example

Let's combine everything into a realistic robot status monitoring system. This example shows how publishers, subscribers, and services work together.

### System Architecture

```
┌─────────────────────┐     /robot_status      ┌─────────────────────┐
│   Status Publisher  │ ──────────────────────▶│   Status Monitor    │
│   (publishes state) │                        │ (logs all updates)  │
└─────────────────────┘                        └─────────────────────┘
                                                         │
                                                         │ calls
                                                         ▼
┌─────────────────────┐                        ┌─────────────────────┐
│  Emergency Stop     │◀═══════════════════════│  /emergency_stop    │
│  Service Server     │       [SERVICE]        │  (safety override)  │
└─────────────────────┘                        └─────────────────────┘
```

### Integrated Node Example

Here's a node that combines publishing, subscribing, and services:

```python
#!/usr/bin/env python3
"""
Integrated Robot Controller Example
=====================================
Demonstrates combining topics and services in a single node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool


class RobotController(Node):
    """
    A robot controller that:
    - Publishes status updates (topic)
    - Subscribes to emergency commands (topic)
    - Provides an enable/disable service
    """

    def __init__(self):
        super().__init__('robot_controller')

        # State
        self.enabled = True
        self.status_count = 0

        # Publisher: Send status updates
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # Subscriber: Listen for emergency stops
        self.emergency_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, 10
        )

        # Service: Enable/disable the robot
        self.enable_srv = self.create_service(
            SetBool, 'set_robot_enabled', self.enable_callback
        )

        # Timer: Publish status every second
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('RobotController initialized')

    def publish_status(self):
        """Publish current robot status."""
        msg = String()
        status = "ENABLED" if self.enabled else "DISABLED"
        msg.data = f'[{self.status_count}] Robot status: {status}'

        self.status_pub.publish(msg)
        self.status_count += 1

    def emergency_callback(self, msg):
        """Handle emergency stop signals."""
        if msg.data:  # True = emergency stop triggered
            self.enabled = False
            self.get_logger().warn('EMERGENCY STOP RECEIVED!')

    def enable_callback(self, request, response):
        """Handle enable/disable service requests."""
        self.enabled = request.data
        response.success = True
        response.message = f'Robot {"enabled" if self.enabled else "disabled"}'

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Testing the Integrated System

```bash
# Terminal 1: Run the controller
python3 robot_controller.py

# Terminal 2: Monitor status
ros2 topic echo /robot_status

# Terminal 3: Disable the robot via service
ros2 service call /set_robot_enabled std_srvs/srv/SetBool "{data: false}"

# Terminal 4: Trigger emergency stop via topic
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

This pattern—combining topics for streaming data with services for commands—is fundamental to real robot systems.

:::info Design Pattern
Use topics for continuous data (sensor streams, status updates) and services for discrete commands (enable/disable, calibrate, reset). This matches how the nervous system separates continuous sensory flow from triggered reflexes.
:::

## Summary

In this chapter, you learned to write Python code that interfaces with ROS 2:

**Key Concepts:**

- **rclpy** is the Python client library for ROS 2
  - Initialize with `rclpy.init()`, run with `rclpy.spin()`, clean up with `rclpy.shutdown()`
  - Subclass `Node` to create your own nodes

- **Publishers** send messages to topics
  - Use `create_publisher(msg_type, topic_name, queue_size)`
  - Call `publish(msg)` to send messages
  - Typically driven by timers for periodic updates

- **Subscribers** receive messages from topics
  - Use `create_subscription(msg_type, topic_name, callback, queue_size)`
  - Callback is called automatically when messages arrive
  - No polling needed—ROS 2 handles message delivery

- **Service Servers** handle requests and return responses
  - Use `create_service(srv_type, service_name, callback)`
  - Callback receives request, fills response, and returns it
  - Must return the response object

- **Service Clients** send requests and receive responses
  - Use `create_client(srv_type, service_name)`
  - Always `wait_for_service()` before calling
  - Use `call_async()` and `spin_until_future_complete()` for synchronous calls

**Nervous System Parallels:**
- Publishers = Sensory neurons sending signals
- Subscribers = Motor neurons receiving commands
- Services = Reflex arcs with guaranteed responses
- `rclpy.spin()` = Staying conscious and responsive

**What's Next:**

Now that you can write Python code to control ROS 2 nodes, you're ready to describe robot structure. In Chapter 4, we'll explore URDF (Unified Robot Description Format)—how humanoid robots define their physical bodies in software.

---

:::tip Next Chapter
Continue to [Chapter 4: Understanding URDF](./urdf-humanoid-robots) to learn how robot bodies are described in software.
:::
