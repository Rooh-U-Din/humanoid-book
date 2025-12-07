"""
Gazebo Warehouse Simulation Launch File

Launches Gazebo with warehouse world and spawns a simple differential drive robot.

Author: Physical AI & Humanoid Robotics Course
License: MIT
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to this package
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.world')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch Gazebo with warehouse world
        gazebo,

        # Note: Robot spawning requires URDF/SDF model
        # For now, this launches the empty warehouse
        # Add spawn_entity.py node here to spawn robot
    ])
