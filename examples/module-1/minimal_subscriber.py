#!/usr/bin/env python3
"""
Minimal ROS 2 Subscriber Example
=================================

This example demonstrates the subscriber side of the publish/subscribe pattern.
Think of this as a "motor neuron" or "processing center" in our robotic nervous
system - it receives signals from sensory neurons and can act on them.

The Nervous System Analogy:
- Subscriber = Motor neuron or brain region receiving signals
- Topic = Neural pathway (shared with publisher)
- Callback = Neural response to incoming signal

Requirements:
- ROS 2 Humble or Iron
- Python 3.10+
- rclpy package

Usage:
    ros2 run your_package minimal_subscriber
    # Or run directly:
    python3 minimal_subscriber.py

Note: Run minimal_publisher.py in another terminal to see messages.

Author: AI-Native Textbook Project
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal ROS 2 subscriber node.

    This node subscribes to a topic and processes incoming messages.
    In our nervous system analogy, this is like a motor neuron that
    receives commands and triggers responses.

    Attributes:
        subscription: The subscription object that receives messages
    """

    def __init__(self):
        """
        Initialize the MinimalSubscriber node.

        Steps performed:
        1. Initialize the parent Node class with our node name
        2. Create a subscription to the 'robot_status' topic
        3. Register our callback function to handle incoming messages
        """
        # Call parent constructor with node name
        super().__init__('minimal_subscriber')

        # Create a subscription
        # Parameters:
        #   - msg_type: The message type we expect (String)
        #   - topic: The topic name to subscribe to ('robot_status')
        #   - callback: Function to call when a message arrives
        #   - qos_profile: Queue size (10 messages)
        self.subscription = self.create_subscription(
            String,                    # Message type
            'robot_status',            # Topic name
            self.listener_callback,    # Callback function
            10                         # QoS queue depth
        )

        # Prevent unused variable warning
        self.subscription  # noqa: B018

        # Log that we've started
        self.get_logger().info('MinimalSubscriber node initialized')
        self.get_logger().info('Listening for messages on "robot_status" topic...')

    def listener_callback(self, msg):
        """
        Callback function executed when a message is received.

        This function is called automatically by ROS 2 whenever a new
        message arrives on the subscribed topic. In our nervous system
        analogy, this is the neural response - what happens when a
        signal reaches its destination.

        Args:
            msg: The received String message
        """
        # Process the incoming message
        # In a real robot, this might trigger motor movements,
        # update internal state, or make decisions
        self.get_logger().info(f'Received: "{msg.data}"')

        # Example: You could add processing logic here
        # For instance, parsing the message and taking action:
        # if 'error' in msg.data.lower():
        #     self.handle_error(msg.data)


def main(args=None):
    """
    Main entry point for the minimal subscriber.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates our subscriber node
    3. Spins (keeps the node running and listening)
    4. Cleans up when done

    The spin() function keeps our node alive and ensures that
    incoming messages are processed by our callback.
    """
    # Initialize rclpy - required before creating any nodes
    rclpy.init(args=args)

    # Create an instance of our subscriber node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Spin the node - this keeps it alive and processing callbacks
        # Our callback will be called whenever a message arrives
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_subscriber.get_logger().info('Shutting down...')
    finally:
        # Clean up: destroy the node and shutdown rclpy
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
