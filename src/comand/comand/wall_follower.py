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

        # Estados
        self.state = "search"

        # Distância desejada à parede
        self.desired_distance = 0.5

        # PID
        self.prev_error = 0.0

        self.get_logger().info("Wall follower iniciado")

    # =========================
    # Ponto mais próximo
    # =========================
    def get_closest(self, msg):
        min_dist = msg.range_max
        min_angle = None

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            if r < min_dist:
                min_dist = r
                angle = msg.angle_min + i * msg.angle_increment
                min_angle = angle

        return min_dist, min_angle

    # =========================
    # Distância à direita
    # =========================
    def get_right_distance(self, msg):
        values = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            # setor da direita
            if -110 <= angle_deg <= -70:
                values.append(r)

        if not values:
            return msg.range_max

        return min(values)

    # =========================
    # CALLBACK
    # =========================
    def scan_callback(self, msg):

        closest_dist, closest_angle = self.get_closest(msg)
        right = self.get_right_distance(msg)

        cmd = Twist()

        # =========================
        # SEARCH
        # =========================
        if self.state == "search":

            if closest_angle is None:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.4
            else:
                self.state = "approach"
                self.get_logger().info("Parede detectada → APPROACH")

        # =========================
        # APPROACH
        # =========================
        elif self.state == "approach":

            if closest_angle is None:
                self.state = "search"
                return

            angle_error = closest_angle

            # alinhar com a parede
            if abs(angle_error) > math.radians(5):
                cmd.linear.x = 0.0
                cmd.angular.z = -1.0 * angle_error

            else:
                # avançar até 0.5m
                if closest_dist > self.desired_distance:
                    cmd.linear.x = 0.15
                    cmd.angular.z = 0.0
                else:
                    self.state = "follow"
                    self.get_logger().info("Distância atingida → FOLLOW")

        # =========================
        # FOLLOW
        # =========================
        elif self.state == "follow":

            if closest_angle is None:
                self.state = "search"
                return

            # erro da distância
            error = self.desired_distance - right
            d_error = error - self.prev_error
            self.prev_error = error

            # ganhos (ajustados)
            Kp = 0.6
            Kd = 0.2

            angular = Kp * error + Kd * d_error

            # 🚨 CORREÇÃO DO SINAL (ESSENCIAL)
            angular = -angular

            # evitar colisão frontal
            front = closest_dist if abs(closest_angle) < math.radians(20) else 10.0

            if front < 0.35:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
            else:
                cmd.linear.x = 0.12
                cmd.angular.z = angular

                # saturação
                if cmd.angular.z > 0.6:
                    cmd.angular.z = 0.6
                elif cmd.angular.z < -0.6:
                    cmd.angular.z = -0.6

        # Debug
        self.get_logger().info(
            f"STATE={self.state} | closest={closest_dist:.2f} | right={right:.2f}"
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