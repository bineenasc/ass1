import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # Subscribe to LIDAR
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publish velocity
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Robot state
        self.state = "search"

        self.get_logger().info("Wall follower node started")

    def scan_callback(self, msg):
        ranges = list(msg.ranges)

        # Replace invalid values
        ranges = [r if r != float('inf') else 10.0 for r in ranges]

        n = len(ranges)

        # Define regions
        front = min(ranges[0:int(n*0.05)] + ranges[int(n*0.95):])
        right = min(ranges[int(n*0.7):int(n*0.85)])

        cmd = Twist()

        # =========================
        # STATE MACHINE
        # =========================

        # 🔍 SEARCH: move forward until wall detected
        if self.state == "search":
            if front < 1.0 or right < 1.0:
                self.state = "approach"
                self.get_logger().info("Wall detected → switching to APPROACH")
            else:
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

        # 🧱 APPROACH: move toward wall
        elif self.state == "approach":
            if right < 0.6:
                self.state = "follow"
                self.get_logger().info("Close to wall → switching to FOLLOW")
            else:
                cmd.linear.x = 0.15
                cmd.angular.z = -0.3  # turn right

        # 🔁 FOLLOW: follow wall smoothly
        elif self.state == "follow":

            # Obstacle ahead → turn left
            if front < 0.4:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.4

            else:
                desired_distance = 0.5
                error = desired_distance - right

                cmd.linear.x = 0.2
                cmd.angular.z = 0.8 * error  # smooth control

        # Debug info
        self.get_logger().info(
            f"STATE={self.state} | front={front:.2f}, right={right:.2f}"
        )

        # Publish command
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()