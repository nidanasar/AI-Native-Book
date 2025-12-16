#!/usr/bin/env python3
"""
Simple ROS 2 Service Server Example
=====================================

This example demonstrates the service (request/response) pattern in ROS 2.
Think of this as a "reflex arc" in our robotic nervous system - it waits
for a specific stimulus and provides a guaranteed response.

The Nervous System Analogy:
- Service Server = Reflex center (spinal cord or brainstem)
- Service Client = Stimulus trigger point
- Request = Incoming stimulus
- Response = Reflex action/feedback

Key Difference from Topics:
- Topics: One-way, continuous data flow (like sensory streaming)
- Services: Two-way, on-demand request/response (like reflexes)

Requirements:
- ROS 2 Humble or Iron
- Python 3.10+
- rclpy package
- example_interfaces package (standard with ROS 2)

Usage:
    ros2 run your_package simple_service
    # Or run directly:
    python3 simple_service.py

Test with:
    ros2 service call /calculate_joint_position example_interfaces/srv/AddTwoInts "{a: 10, b: 5}"

Author: AI-Native Textbook Project
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleService(Node):
    """
    A simple ROS 2 service server node.

    This node provides a service that adds two integers - a simplified
    example that could represent calculating a joint position from
    two sensor readings in a real robot.

    In our nervous system analogy, this is like a reflex arc:
    - Receives a stimulus (request with two numbers)
    - Processes it (adds them together)
    - Returns a response (the sum)

    Attributes:
        srv: The service object that handles incoming requests
    """

    def __init__(self):
        """
        Initialize the SimpleService node.

        Steps performed:
        1. Initialize the parent Node class with our node name
        2. Create a service with a callback to handle requests
        """
        # Call parent constructor with node name
        super().__init__('simple_service')

        # Create a service
        # Parameters:
        #   - srv_type: The service type (AddTwoInts)
        #   - srv_name: The service name ('calculate_joint_position')
        #   - callback: Function to call when a request arrives
        self.srv = self.create_service(
            AddTwoInts,                    # Service type
            'calculate_joint_position',    # Service name
            self.calculate_callback        # Callback function
        )

        # Log that we're ready
        self.get_logger().info('SimpleService node initialized')
        self.get_logger().info('Service "calculate_joint_position" is ready')

    def calculate_callback(self, request, response):
        """
        Callback function executed when a service request is received.

        This function processes the request and fills in the response.
        Unlike topic callbacks, service callbacks MUST return a response.

        In our nervous system analogy:
        - request.a and request.b are like two sensory inputs
        - response.sum is like the motor command output
        - This mimics how the nervous system integrates multiple
          inputs to produce a coordinated response

        Args:
            request: The incoming request with 'a' and 'b' integers
            response: The response object to fill with 'sum'

        Returns:
            The filled response object
        """
        # Process the request
        # In a real robot, this might calculate:
        # - Target joint angle from two sensor readings
        # - Combined force from two pressure sensors
        # - Merged position estimate from two localization sources
        response.sum = request.a + request.b

        # Log the operation
        self.get_logger().info(
            f'Request received: {request.a} + {request.b} = {response.sum}'
        )

        # Return the response - this is REQUIRED for services
        return response


def main(args=None):
    """
    Main entry point for the simple service.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates our service node
    3. Spins (keeps the node running and waiting for requests)
    4. Cleans up when done

    The service will wait indefinitely for requests, processing
    each one as it arrives - just like a reflex arc that's always
    ready to respond to stimuli.
    """
    # Initialize rclpy - required before creating any nodes
    rclpy.init(args=args)

    # Create an instance of our service node
    simple_service = SimpleService()

    try:
        # Spin the node - this keeps it alive and processing requests
        rclpy.spin(simple_service)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        simple_service.get_logger().info('Shutting down...')
    finally:
        # Clean up: destroy the node and shutdown rclpy
        simple_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
