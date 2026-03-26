from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('comand')

    world_path = os.path.join(pkg_path, 'worlds', 'building_robot.sdf')
    models_path = os.path.join(pkg_path, 'models')

    return LaunchDescription([

        # Tell Gazebo where to find models
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        )

    ])