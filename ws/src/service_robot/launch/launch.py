#! /usr/bin/env python3 

# Libraries importing
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Utilities importing


# Environment variables definition


# Launch description
def generate_launch_description():
    # Nodes
    robot = Node(
        package='service_robot',
        executable='robot_controller',
        name='robot',
        output='screen',
    )
    state_machine = Node(
        package='service_robot',
        executable='state_machine',
        name='state_machine',
        output='screen',
    )
    log = Node(
        package='service_robot',
        executable='logger',
        name='logger',
        output='screen',
    )

    # Files 

    # Executors 
    rvizz = ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=True', 'map:=../assets/maps/maps.yaml'],
            name='navigator_ros2',
            output='screen'
    )

    # Launch description
    return LaunchDescription([
        log,
        robot,
        state_machine,
        rvizz
    ])

if __name__ == "__main__":
    generate_launch_description()