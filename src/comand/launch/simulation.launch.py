from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from ament_index_python.packages import get_package_share_directory
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



    # -------------------------#
    # SPAWN ROBOT COORDINATES #
    # -------------------------#

    resolution = 0.05  # meters
    margin = 0.30 * 2 / 3  # safety margin

    mesh_path = os.path.join(models_path, 'cinco', 'meshes', 'cinco.stl')
    mesh = trimesh.load(mesh_path)

    # Bounding box
    bounds = mesh.bounds
    min_x, min_y, min_z = bounds[0]
    max_x, max_y, max_z = bounds[1]

    # Expand bounds slightly for safety
    min_x -= margin
    max_x += margin
    min_y -= margin
    max_y += margin

    # Create grid
    x_vals = np.arange(min_x, max_x, resolution)
    y_vals = np.arange(min_y, max_y, resolution)

    # Generate candidate points
    points = np.array([[x, y, 0] for x in x_vals for y in y_vals])

    # Check which points are inside the 5 (mesh)
    inside = mesh.contains(points)

    # Keep only valid (outside 5 mesh)
    valid_points = [
        (p[0], p[1])
        for p, is_inside in zip(points, inside)
        if not is_inside
    ]

    # Safety fallback ^just in case^
    if not valid_points:
        raise RuntimeError("No valid spawn points found!")

    # random spawn point
    spawn_x, spawn_y = random.choice(valid_points)
    spawn_z = 0.0

    print(f"Spawning robot at: x={spawn_x}, y={spawn_y}, z={spawn_z}")
    

    return LaunchDescription([

        # tell Gazebo where to find models
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        # start gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # spawn robot - use a delay in case it is needed
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-name', 'diff_robot',
                        '-x', str(spawn_x),
                        '-y', str(spawn_y),
                        '-z', str(spawn_z),
                        '-file', robot_model
                    ],
                    output='screen'
                )
            ]
        ),

    ])

