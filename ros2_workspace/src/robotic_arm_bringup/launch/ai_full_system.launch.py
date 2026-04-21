#!/usr/bin/env python3
"""Master AI System Launch File — The Full Autonomous Stack.

This is the ultimate launch file that brings the FULL project to life.
It coordinates Vision, Brain, Kinematics (MoveIt), and Hardware communication.

Launches:
  1. MoveIt 2 (demo.launch.py) → RViz, KDL Kinematics, Mock Hardware, State Publisher
  2. Serial Bridge             → Arduino Communication (relays MoveIt's joint_states)
  3. Vision Perception         → Camera & AI Object Tracking
  4. Task Orchestrator         → FSM Logic (The Brain)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Launch Arguments
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

    # 2. MoveIt 2 Integration (The Digital Twin Core)
    pkg_moveit_config = get_package_share_directory('robotic_arm_moveit_config')
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'demo.launch.py')
        )
    )

    # 3. Hardware Communication Node (Digital Twin Relay)
    serial_bridge_node = Node(
        package='robotic_arm_hardware',
        executable='node_serial_bridge',
        name='node_serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'rate_hz': 40
        }]
    )

    # 4. Vision Perception (Camera feed & AI)
    vision_node = Node(
        package='robotic_arm_vision',
        executable='vision_perception_node',
        name='vision_perception_node',
        output='screen'
    )

    # 5. Task Orchestrator (FSM Brain)
    brain_node = Node(
        package='robotic_arm_brain',
        executable='task_orchestrator_node',
        name='task_orchestrator_node',
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        moveit_demo_launch,
        serial_bridge_node,
        vision_node,
        brain_node
    ])
