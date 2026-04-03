from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('comand')

    world_path = os.path.join(pkg_path, 'worlds', 'building_robot.sdf')
    models_path = os.path.join(pkg_path, 'models')

    return LaunchDescription([

        # Tell Gazebo where models are
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        # Start Gazebo
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
        )

    ])