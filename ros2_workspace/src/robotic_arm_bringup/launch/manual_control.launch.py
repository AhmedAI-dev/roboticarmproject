#!/usr/bin/env python3
"""Manual Control Launch — RViz2 + GUI Teleop + Hardware Bridge.

This mode allows you to control the physical robotic arm using the slider GUI.
It bridges the ROS 2 joint states to the Arduino firmware.

Launches:
  1. robot_state_publisher  → Broadcasts TF tree from URDF
  2. rviz2                  → 3D visualization window
  3. node_gui_teleop        → Slider GUI for manual joint control
  4. node_serial_bridge     → Communicates with Arduino hardware
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Paths
    pkg_description = get_package_share_directory('robotic_arm_description')
    pkg_visualization = get_package_share_directory('robotic_arm_visualization')
    
    urdf_file = os.path.join(pkg_description, 'urdf', 'arm_robot.urdf')
    rviz_file = os.path.join(pkg_visualization, 'rviz', 'view_arm.rviz')

    # 2. Launch Arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB port connected to the Arduino'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    # 3. Nodes
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file]
    )

    gui_teleop_node = Node(
        package='robotic_arm_teleop',
        executable='node_gui_teleop',
        name='node_gui_teleop',
        output='screen'
    )

    serial_bridge_node = Node(
        package='robotic_arm_hardware',
        executable='node_serial_bridge',
        name='node_serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'rate_hz': 20
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        robot_state_publisher_node,
        rviz_node,
        gui_teleop_node,
        serial_bridge_node
    ])
