"""
Spawn Robot in Gazebo - Launch File

This launch file demonstrates how to:
1. Start Gazebo with a custom world
2. Publish robot description (URDF)
3. Spawn the robot into Gazebo
4. Set up ros_gz_bridge for ROS 2 communication

Usage:
    ros2 launch my_robot_gazebo spawn_robot.launch.py

Prerequisites:
    - ros-humble-ros-gz (or ros-iron-ros-gz)
    - Your robot's URDF package

Note: Update the package names and paths for your specific robot.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # For this example, we'll use inline URDF
    # In practice, load from your robot description package:
    # urdf_file = os.path.join(
    #     get_package_share_directory('my_humanoid_description'),
    #     'urdf', 'simple_humanoid.urdf'
    # )

    # Simple robot description for demonstration
    robot_description = """<?xml version="1.0"?>
<robot name="demo_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 1.0"/>
      </material>
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
</robot>
"""

    # World file path (use the simple_world.sdf from this directory)
    world_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        'simple_world.sdf'
    )

    # If world file doesn't exist, use empty world
    if not os.path.exists(world_file):
        world_file = 'empty.sdf'

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # Start Gazebo simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',  # -r starts in running state
            }.items(),
        ),

        # Publish robot description to /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            arguments=[
                '-name', 'simple_humanoid',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '1.0',  # Spawn above ground to avoid collision issues
            ],
            output='screen',
        ),

        # Bridge between Gazebo and ROS 2
        # Format: <ros_topic>@<ros_type>@<gz_type>
        # [ means Gazebo->ROS, ] means ROS->Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                # Clock: Gazebo -> ROS 2
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states: Gazebo -> ROS 2
                '/world/simple_world/model/simple_humanoid/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
