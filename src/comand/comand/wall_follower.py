import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.state = "search"
        self.desired_distance = 0.5
        self.prev_error = 0.0

        self.get_logger().info("Wall follower iniciado")

    def normalize_deg(self, angle_deg):
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg

    def logical_to_scan_deg(self, logical_deg):
        # corrige inversão frente/trás
        return self.normalize_deg(logical_deg + 180)

    def angle_in_sector(self, angle_deg, min_deg, max_deg):
        angle_deg = self.normalize_deg(angle_deg)
        min_deg = self.normalize_deg(min_deg)
        max_deg = self.normalize_deg(max_deg)

        if min_deg <= max_deg:
            return min_deg <= angle_deg <= max_deg
        else:
            return angle_deg >= min_deg or angle_deg <= max_deg

    def get_closest(self, msg):
        min_dist = msg.range_max
        min_angle = None

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            if r < min_dist:
                min_dist = r
                min_angle = msg.angle_min + i * msg.angle_increment

        return min_dist, min_angle

    def get_sector_distance(self, msg, logical_min_deg, logical_max_deg, k=5):
        scan_min_deg = self.logical_to_scan_deg(logical_min_deg)
        scan_max_deg = self.logical_to_scan_deg(logical_max_deg)

        values = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            scan_angle = msg.angle_min + i * msg.angle_increment
            scan_angle_deg = math.degrees(scan_angle)

            if self.angle_in_sector(scan_angle_deg, scan_min_deg, scan_max_deg):
                values.append(r)

        if not values:
            return msg.range_max

        values.sort()
        values = values[:min(k, len(values))]
        return sum(values) / len(values)

    def scan_callback(self, msg):

        closest_dist, closest_angle = self.get_closest(msg)

        # setores lógicos do robô
        front = self.get_sector_distance(msg, -15, 15)
        front_right = self.get_sector_distance(msg, -55, -15)
        right = self.get_sector_distance(msg, -110, -50)
        front_left = self.get_sector_distance(msg, 15, 45)  

        cmd = Twist()

        if self.state == "search":

            if closest_angle is None:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.35
            else:
                self.state = "approach"
                self.get_logger().info("Parede detectada → APPROACH")

        elif self.state == "approach":

            if closest_angle is None:
                self.state = "search"
                return

            angle_error = closest_angle

            if closest_dist <= self.desired_distance:
                self.state = "follow"
                self.prev_error = 0.0
                self.get_logger().info("Distância atingida → FOLLOW")
            else:
                cmd.linear.x = 0.12
                cmd.angular.z = 0.8 * angle_error

                if cmd.angular.z > 0.5:
                    cmd.angular.z = 0.5
                elif cmd.angular.z < -0.5:
                    cmd.angular.z = -0.5

        elif self.state == "follow":

            if front < 1.5:
                cmd.linear.x = 0.03
                cmd.angular.z = 0.60

            elif front_right < 1.50:
                cmd.linear.x = 0.05
                cmd.angular.z = 0.45
                
            elif front_left < 1.50:
                cmd.linear.x = 0.08
                cmd.angular.z = -0.10  # leve ajuste à direita

            elif right > 1.50:
                cmd.linear.x = 0.08
                cmd.angular.z = -0.25

            else:
                error = self.desired_distance - right
                d_error = error - self.prev_error
                self.prev_error = error

                Kp = 1.2
                Kd = 0.25

                angular = -(Kp * error + Kd * d_error)

                if angular > 0.40:
                    angular = 0.40
                elif angular < -0.40:
                    angular = -0.40

                cmd.linear.x = 0.10
                cmd.angular.z = angular

        angle_deg = math.degrees(closest_angle) if closest_angle is not None else 0.0

        self.get_logger().info(
            f"STATE={self.state} | "
            f"closest={closest_dist:.2f} | "
            f"angle={angle_deg:.1f} | "
            f"front={front:.2f} | "
            f"front_right={front_right:.2f} | "
            f"right={right:.2f} | "
            f"front_left={front_left:.2f}"
        )

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