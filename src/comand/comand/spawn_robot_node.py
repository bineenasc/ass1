#------------------------------#
# TO RANDOMLLY SPAWN THE ROBOT #
#------------------------------#

import rclpy
from rclpy.node import Node
import random
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from shapely.geometry import Point

from comand.footprint_utils import compute_footprint


class SpawnRobotNode(Node):
    def __init__(self):
        super().__init__('spawn_robot')

        pkg_path = get_package_share_directory('comand')
        mesh_path = os.path.join(pkg_path, 'models', 'cinco', 'meshes', 'cinco.stl') 

        footprint = compute_footprint(mesh_path)

        minx, miny, maxx, maxy = footprint.bounds

        expansion_factor = 2.0 

        expanded_minx = minx - (maxx - minx) * (expansion_factor - 1) / 2
        expanded_maxx = maxx + (maxx - minx) * (expansion_factor - 1) / 2
        expanded_miny = miny - (maxy - miny) * (expansion_factor - 1) / 2
        expanded_maxy = maxy + (maxy - miny) * (expansion_factor - 1) / 2

        padding = 2.0
        expanded_minx -= padding
        expanded_maxx += padding
        expanded_miny -= padding
        expanded_maxy += padding


        for _ in range(1000):
            x = random.uniform(expanded_minx, expanded_maxx)
            y = random.uniform(expanded_miny, expanded_maxy)

            if not footprint.contains(Point(x, y)):
                spawn_x, spawn_y = x, y
                break
        else:
            raise RuntimeError("No valid spawn found   :(")

        self.get_logger().info(f"Spawn at: {spawn_x}, {spawn_y}")

        self.spawn_robot(spawn_x, spawn_y)

        self.modify_world_file(spawn_x,spawn_y)
        return
    
    def modify_world_file(self, x, y):

        world_file_path = os.path.join(get_package_share_directory('comand'), 'worlds', 'building_robot.sdf')

        tree = ET.parse(world_file_path)
        root = tree.getroot()

        for include in root.findall(".//include"):
            if include.find("name").text == "diff_robot":
                pose_element = include.find("pose")
                if pose_element is not None:

                    pose_element.text = f"{x} {y} 0 0 0 0"

        tree.write(world_file_path)

        self.spawn_robot(x, y)
        return


    def spawn_robot(self, x, y):

        pkg_path = get_package_share_directory('comand')
        robot_model = os.path.join(pkg_path, 'models', 'diff_robot', 'model.sdf')

        os.system(
            f"ros2 run ros_gz_sim create "
            f"-name diff_robot "
            f"-x {x} -y {y} -z 0.0 "
            f"-file {robot_model}"
        )
        print("-----should be working-----")
        return


def main():
    rclpy.init()
    node = SpawnRobotNode()
    rclpy.shutdown()