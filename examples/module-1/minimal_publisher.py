#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example
================================

This example demonstrates the fundamental publish/subscribe pattern in ROS 2.
Think of this as a "sensory neuron" in our robotic nervous system - it sends
signals (messages) to other parts of the system.

The Nervous System Analogy:
- Publisher = Sensory neuron sending signals
- Topic = Neural pathway (the route messages travel)
- Message = Neural impulse (the actual information)

Requirements:
- ROS 2 Humble or Iron
- Python 3.10+
- rclpy package

Usage:
    ros2 run your_package minimal_publisher
    # Or run directly:
    python3 minimal_publisher.py

Author: AI-Native Textbook Project
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node.

    This node publishes a simple string message to a topic at regular intervals.
    In our nervous system analogy, this is like a sensory receptor that
    continuously sends information about the environment.

    Attributes:
        publisher_: The publisher object that sends messages
        timer: A timer that triggers message publication
        count: A counter to track how many messages have been sent
    """

    def __init__(self):
        """
        Initialize the MinimalPublisher node.

        Steps performed:
        1. Initialize the parent Node class with our node name
        2. Create a publisher for String messages on 'robot_status' topic
        3. Set up a timer to call our callback every 0.5 seconds
        4. Initialize a message counter
        """
        # Call parent constructor with node name
        # This registers our node with the ROS 2 system
        super().__init__('minimal_publisher')

        # Create a publisher
        # Parameters:
        #   - msg_type: The message type (String from std_msgs)
        #   - topic: The topic name ('robot_status')
        #   - qos_profile: Queue size (10 messages)
        self.publisher_ = self.create_publisher(
            String,           # Message type
            'robot_status',   # Topic name
            10                # QoS queue depth
        )

        # Create a timer that fires every 0.5 seconds
        # This is like setting up a heartbeat for our sensory neuron
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track published messages
        self.count = 0

        # Log that we've started
        self.get_logger().info('MinimalPublisher node initialized')

    def timer_callback(self):
        """
        Callback function executed by the timer.

        This function:
        1. Creates a new String message
        2. Fills it with data (including a counter)
        3. Publishes the message to our topic
        4. Logs what was sent

        In the nervous system analogy, this is like a neuron firing -
        it generates and transmits a signal at regular intervals.
        """
        # Create a new message object
        msg = String()

        # Fill the message with data
        msg.data = f'Robot status update #{self.count}: All systems operational'

        # Publish the message
        # This sends it to all nodes subscribed to 'robot_status'
        self.publisher_.publish(msg)

        # Log what we published (useful for debugging)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment our counter
        self.count += 1


def main(args=None):
    """
    Main entry point for the minimal publisher.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates our publisher node
    3. Spins (keeps the node running and processing)
    4. Cleans up when done

    The spin() function is crucial - it keeps our node alive and
    ensures callbacks are processed. Without it, the node would
    exit immediately.
    """
    # Initialize rclpy - required before creating any nodes
    rclpy.init(args=args)

    # Create an instance of our publisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Spin the node - this keeps it alive and processing callbacks
        # Think of this as keeping our "neuron" active and firing
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_publisher.get_logger().info('Shutting down...')
    finally:
        # Clean up: destroy the node and shutdown rclpy
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
