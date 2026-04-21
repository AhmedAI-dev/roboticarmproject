#!/usr/bin/env python3
"""Visualization Launch — Simulation Mode (No Hardware).

This mode allows you to preview the 3D model in RViz and test the joints 
visually using the slider GUI without needing a physical robot.

Launches:
  1. robot_state_publisher  → Broadcasts TF tree from URDF
  2. rviz2                  → 3D visualization window
  3. node_gui_teleop        → Slider GUI for manual joint control
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths
    pkg_description = get_package_share_directory('robotic_arm_description')
    pkg_visualization = get_package_share_directory('robotic_arm_visualization')
    
    urdf_file = os.path.join(pkg_description, 'urdf', 'arm_robot.urdf')
    rviz_file = os.path.join(pkg_visualization, 'rviz', 'view_arm.rviz')

    # 2. Load URDF
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description_content = f.read()

    # 3. Nodes
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

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        gui_teleop_node
    ])
