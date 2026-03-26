from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', 'worlds/building_robot.sdf'],
            output='screen'
        )
    ])

from glob import glob
import os

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/comand']),
    ('share/comand', ['package.xml']),
    (os.path.join('share', 'comand', 'launch'), glob('launch/*.launch.py')),
],