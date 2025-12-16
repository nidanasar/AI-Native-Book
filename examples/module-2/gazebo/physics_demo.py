#!/usr/bin/env python3
"""
Physics Demo - Joint Control Example

This script demonstrates how to control a simulated robot's joints
in Gazebo through ROS 2. It shows:
1. Publishing joint commands
2. Subscribing to joint states
3. Basic motion patterns

Usage:
    # First, launch the simulation:
    ros2 launch my_robot_gazebo spawn_robot.launch.py

    # Then run this demo:
    ros2 run my_robot_gazebo physics_demo.py

Prerequisites:
    - Simulation running with ros_gz_bridge active
    - Robot spawned with joint controllers configured
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math


class PhysicsDemo(Node):
    """
    Demonstrates physics simulation by controlling robot joints
    and observing their states.
    """

    def __init__(self):
        super().__init__('physics_demo')

        # Parameters
        self.declare_parameter('joint_name', 'left_shoulder_joint')
        self.declare_parameter('amplitude', 1.0)  # radians
        self.declare_parameter('frequency', 0.5)  # Hz

        self.joint_name = self.get_parameter('joint_name').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value

        # Publishers for joint commands
        # Note: Topic name depends on your Gazebo plugin configuration
        self.joint_cmd_pub = self.create_publisher(
            Float64,
            f'/{self.joint_name}_cmd',
            10
        )

        # Subscriber for joint states feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for sending sinusoidal commands
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        # State tracking
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.command_count = 0

        self.get_logger().info(
            f'Physics Demo started. Controlling joint: {self.joint_name}'
        )
        self.get_logger().info(
            f'Motion: amplitude={self.amplitude} rad, frequency={self.frequency} Hz'
        )

    def timer_callback(self):
        """Generate sinusoidal joint command."""
        # Calculate elapsed time
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        # Sinusoidal motion: position = A * sin(2 * pi * f * t)
        target_position = self.amplitude * math.sin(
            2 * math.pi * self.frequency * elapsed
        )

        # Publish command
        msg = Float64()
        msg.data = target_position
        self.joint_cmd_pub.publish(msg)

        # Log periodically
        self.command_count += 1
        if self.command_count % 100 == 0:
            self.get_logger().info(
                f'Command: {target_position:.3f} rad | '
                f'Actual: {self.current_position:.3f} rad | '
                f'Error: {abs(target_position - self.current_position):.4f} rad'
            )

    def joint_state_callback(self, msg: JointState):
        """Process incoming joint state feedback."""
        try:
            # Find our joint in the message
            if self.joint_name in msg.name:
                idx = msg.name.index(self.joint_name)
                self.current_position = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.current_velocity = msg.velocity[idx]
        except (ValueError, IndexError) as e:
            # Joint not found in this message - this is normal if
            # the simulation publishes states for multiple robots
            pass


class MultiJointDemo(Node):
    """
    Alternative demo: Control multiple joints simultaneously.
    Demonstrates coordinated motion patterns.
    """

    def __init__(self):
        super().__init__('multi_joint_demo')

        # Define joints to control
        self.joints = [
            'left_shoulder_joint',
            'left_elbow_joint',
            'right_shoulder_joint',
            'right_elbow_joint',
        ]

        # Create publishers for each joint
        self.publishers = {}
        for joint in self.joints:
            self.publishers[joint] = self.create_publisher(
                Float64,
                f'/{joint}_cmd',
                10
            )

        # Subscriber for state feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.get_logger().info('Multi-Joint Demo started')
        self.get_logger().info(f'Controlling joints: {self.joints}')

    def timer_callback(self):
        """Generate coordinated arm motion."""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        # Example: Waving motion
        # Shoulders move in opposite directions (anti-phase)
        # Elbows bend during the wave

        shoulder_angle = 0.8 * math.sin(2 * math.pi * 0.5 * elapsed)
        elbow_angle = 0.5 + 0.3 * math.sin(2 * math.pi * 1.0 * elapsed)

        commands = {
            'left_shoulder_joint': shoulder_angle,
            'left_elbow_joint': elbow_angle,
            'right_shoulder_joint': -shoulder_angle,  # Opposite
            'right_elbow_joint': elbow_angle,
        }

        # Publish commands
        for joint, angle in commands.items():
            if joint in self.publishers:
                msg = Float64()
                msg.data = angle
                self.publishers[joint].publish(msg)

    def joint_state_callback(self, msg: JointState):
        """Log joint states."""
        # You can add tracking logic here similar to PhysicsDemo
        pass


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Choose which demo to run
    # node = PhysicsDemo()       # Single joint demo
    node = PhysicsDemo()         # Default to single joint

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
