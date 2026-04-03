#------------------------------#
# TO RANDOMLLY SPAWN THE ROBOT #
#------------------------------#

import rclpy
from rclpy.node import Node
import random
import os

from ament_index_python.packages import get_package_share_directory
from shapely.geometry import Point

from comand.footprint_utils import compute_footprint


class SpawnRobotNode(Node):

    def __init__(self):
        super().__init__('spawn_robot')

        pkg_path = get_package_share_directory('comand')
        mesh_path = os.path.join(pkg_path, 'models', 'cinco', 'meshes', 'cinco.stl')

        footprint = compute_footprint(mesh_path)

        minx, miny, maxx, maxy = footprint.bounds # area of map/spawn (5 is inside)

        padding = 2.0 # better if used the actual robot diameter but nhe
        minx -= padding
        maxx += padding
        miny -= padding
        maxy += padding

        # simple random sampling to find valid point
        for _ in range(1000):
            x = random.uniform(minx, maxx)
            y = random.uniform(miny, maxy)

            if not footprint.contains(Point(x, y)):
                spawn_x, spawn_y = x, y
                break
        else:
            raise RuntimeError("No valid spawn found")

        self.get_logger().info(f"Spawn at: {spawn_x}, {spawn_y}")

        # call spawn service
        self.spawn_robot(spawn_x, spawn_y)
        return

    def spawn_robot(self, x, y): # the call spawn service in question
        pkg_path = get_package_share_directory('comand')
        robot_model = os.path.join(pkg_path, 'models', 'diff_robot', 'model.sdf')

        os.system(
            f"ros2 run ros_gz_sim create "
            f"-name diff_robot "
            f"-x {x} -y {y} -z 0.05 "
            f"-file {robot_model}"
        )
        return


def main():
    rclpy.init()
    node = SpawnRobotNode()
    rclpy.shutdown()