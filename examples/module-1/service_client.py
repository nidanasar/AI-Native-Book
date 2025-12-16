#!/usr/bin/env python3
"""
ROS 2 Service Client Example
==============================

This example demonstrates how to call a ROS 2 service from a client node.
Think of this as the "stimulus trigger" in our robotic nervous system -
it initiates a request and waits for the reflex response.

The Nervous System Analogy:
- Service Client = Sensory receptor triggering a reflex
- Service Server = Reflex arc processing center
- Request = Stimulus signal
- Response = Reflex response

Why Use Services Instead of Topics?
- Guaranteed response (you know the server received your request)
- Synchronous when needed (wait for result before continuing)
- Request/response pairs are matched (no message loss ambiguity)

Requirements:
- ROS 2 Humble or Iron
- Python 3.10+
- rclpy package
- example_interfaces package (standard with ROS 2)

Usage:
    # First, start the service server:
    python3 simple_service.py

    # Then run this client:
    python3 service_client.py

Author: AI-Native Textbook Project
License: Apache 2.0
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClient(Node):
    """
    A ROS 2 service client node.

    This node sends requests to a service and handles responses.
    In our nervous system analogy, this is like a sensory receptor
    that triggers a reflex and then processes the response.

    Attributes:
        client: The service client object for sending requests
    """

    def __init__(self):
        """
        Initialize the ServiceClient node.

        Steps performed:
        1. Initialize the parent Node class with our node name
        2. Create a client for the service
        3. Wait for the service to become available
        """
        # Call parent constructor with node name
        super().__init__('service_client')

        # Create a client for the service
        # Parameters:
        #   - srv_type: The service type (must match server)
        #   - srv_name: The service name (must match server)
        self.client = self.create_client(
            AddTwoInts,                  # Service type
            'calculate_joint_position'   # Service name
        )

        # Wait for the service to be available
        # This is important - calling a service that doesn't exist will fail
        self.get_logger().info('Waiting for service to become available...')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'Service not available, waiting...'
            )

        self.get_logger().info('Service is available!')

    def send_request(self, a, b):
        """
        Send a request to the service and wait for response.

        This method demonstrates synchronous service calling:
        1. Create a request object
        2. Fill in the request data
        3. Send the request asynchronously
        4. Wait for the response

        In our nervous system analogy:
        - a and b are sensory inputs (e.g., sensor readings)
        - The response contains the processed result

        Args:
            a: First integer value
            b: Second integer value

        Returns:
            The response from the service
        """
        # Create a request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Send the request asynchronously
        # call_async returns a Future object that will contain the response
        future = self.client.call_async(request)

        # Wait for the response
        # spin_until_future_complete blocks until the Future is done
        rclpy.spin_until_future_complete(self, future)

        # Return the result
        return future.result()


def main(args=None):
    """
    Main entry point for the service client.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates our client node
    3. Sends a request and processes the response
    4. Cleans up when done

    Unlike publisher/subscriber nodes that run indefinitely,
    a service client typically sends a request and exits.
    """
    # Initialize rclpy
    rclpy.init(args=args)

    # Create our client node
    service_client = ServiceClient()

    try:
        # Example values - in a real robot these might come from:
        # - User input
        # - Sensor readings
        # - Planning algorithms
        value_a = 10
        value_b = 5

        # Check for command line arguments
        if len(sys.argv) >= 3:
            value_a = int(sys.argv[1])
            value_b = int(sys.argv[2])

        # Send the request and get the response
        response = service_client.send_request(value_a, value_b)

        # Process the response
        service_client.get_logger().info(
            f'Result: {value_a} + {value_b} = {response.sum}'
        )

    except KeyboardInterrupt:
        service_client.get_logger().info('Interrupted by user')
    except Exception as e:
        service_client.get_logger().error(f'Error: {e}')
    finally:
        # Clean up
        service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
