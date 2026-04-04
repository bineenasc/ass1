from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import numpy as np
import trimesh
import random


def generate_launch_description():

    # Package paths
    pkg_path = get_package_share_directory('comand')

    world_path = os.path.join(pkg_path, 'worlds', 'building_robot.sdf')
    models_path = os.path.join(pkg_path, 'models')
    robot_model = os.path.join(models_path, 'diff_robot', 'model.sdf')



    return LaunchDescription([

        # Tell Gazebo where models are
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        # start gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # Spawn robot AFTER Gazebo starts
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='comand',
                    executable='spawn_robot',
                    output='screen'
                )
            ]
        ),

    ])

