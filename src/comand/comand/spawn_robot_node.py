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
    ''' 
    ---ROS 2 node class---

    '''

    def __init__(self):
        ''' 
        Description:

        ----------
        Input:
        ----------
        Output
        '''
        super().__init__('spawn_robot')

        # get 5 shaped wall package path from the model files
        pkg_path = get_package_share_directory('comand')
        mesh_path = os.path.join(pkg_path, 'models', 'cinco', 'meshes', 'cinco.stl') 

        footprint = compute_footprint(mesh_path)

        #bounding box:
        minx, miny, maxx, maxy = footprint.bounds # area of map/spawn (5 is inside)

        # Expand the area to expansion_factorX the size of the footprint
        expansion_factor = 2.0 # chosen 2 because sound reasonable 

        expanded_minx = minx - (maxx - minx) * (expansion_factor - 1) / 2
        expanded_maxx = maxx + (maxx - minx) * (expansion_factor - 1) / 2
        expanded_miny = miny - (maxy - miny) * (expansion_factor - 1) / 2
        expanded_maxy = maxy + (maxy - miny) * (expansion_factor - 1) / 2

        # padding in case the robot needs some room near the walls
        padding = 2.0
        expanded_minx -= padding
        expanded_maxx += padding
        expanded_miny -= padding
        expanded_maxy += padding


        # random sampling till find valid point
        for _ in range(1000):
            x = random.uniform(expanded_minx, expanded_maxx)
            y = random.uniform(expanded_miny, expanded_maxy)

            # point cant be inside the wall -> when outside found a possible spawn position -> end the loop
            if not footprint.contains(Point(x, y)):
                spawn_x, spawn_y = x, y
                break
        else:
            raise RuntimeError("No valid spawn found   :(")

        self.get_logger().info(f"Spawn at: {spawn_x}, {spawn_y}")

        # call spawn service
        self.spawn_robot(spawn_x, spawn_y)

        # alter/add <pose> to the bulding_robot.sdf
        self.modify_world_file(spawn_x,spawn_y)
        return
    
    def modify_world_file(self, x, y):
        ''' 
        Description:

        ----------
        Input:
        ----------
        Output
        '''
        # path to gazebo world file
        world_file_path = os.path.join(get_package_share_directory('comand'), 'worlds', 'building_robot.sdf')

        # Parse the SDF file
        tree = ET.parse(world_file_path)
        root = tree.getroot()

        # Find the <include> tag for the diff_robot and modify the <pose> element
        for include in root.findall(".//include"):
            if include.find("name").text == "diff_robot":
                pose_element = include.find("pose")
                if pose_element is not None:
                    # Modify the pose element with the random spawn position
                    pose_element.text = f"{x} {y} 0 0 0 0"  # x, y, z, roll, pitch, yaw

        # Write the updated world file back to disk
        tree.write(world_file_path)

        # Spawn the robot with the updated world file
        self.spawn_robot(x, y)
        return


    def spawn_robot(self, x, y): # the call spawn service in question
        ''' 
        Description:

        ----------
        Input:
        ----------
        Output
        '''

        # Get the model path for the robot
        pkg_path = get_package_share_directory('comand')
        robot_model = os.path.join(pkg_path, 'models', 'diff_robot', 'model.sdf')

        # Call the spawn service using ros2 run
        os.system(
            f"ros2 run ros_gz_sim create "
            f"-name diff_robot "
            f"-x {x} -y {y} -z 0.0 "  # the randomly sampled coords and z=0(ground)
            f"-file {robot_model}"
        )
        print("-----should be working-----")
        return


def main():
    rclpy.init()
    node = SpawnRobotNode()
    rclpy.shutdown()