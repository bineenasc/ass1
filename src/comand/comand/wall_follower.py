import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class PID:
    def __init__(self, kp, ki, kd, output_min, output_max, anti_windup=True):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.anti_windup = anti_windup
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, error, current_time):
        dt = 0.05 if self._prev_time is None else max(current_time - self._prev_time, 1e-6)
        self._prev_time = current_time
        p = self.kp * error
        self._integral += error * dt
        i = self.ki * self._integral
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        output = max(self.output_min, min(self.output_max, p + i + d))
        if self.anti_windup and (output == self.output_min or output == self.output_max):
            self._integral -= error * dt
        return output


def fit_circle_lstsq(points):
    if len(points) < 3:
        return None
    pts = np.array(points)
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    try:
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = result[0], result[1]
    c = result[2]
    r2 = c + cx ** 2 + cy ** 2
    if r2 <= 0:
        return None
    radius = math.sqrt(r2)
    dists = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    residual = float(np.sqrt(np.mean((dists - radius) ** 2)))
    return cx, cy, radius, residual


class WallFollower(Node):
    # ── Circle-fitting detection ────────────────────────────────────────────
    # Sector do laser (em graus lógicos, 0=frente, +90=esquerda) usado
    # para o fitting. Abrange a zona onde a cavidade aparece.
    CIRCLE_SECTOR_LO   = 20     # deg  (diagonal frontal-esq)
    CIRCLE_SECTOR_HI   = 160    # deg  (diagonal traseira-esq)

    CIRCLE_MIN_POINTS  = 12     # mínimo de pontos para tentar fit
    CIRCLE_MAX_RESIDUAL= 0.10   # m  — aumentado: parede não é perfeita
    CIRCLE_MIN_RADIUS  = 0.50   # m
    CIRCLE_MAX_RADIUS  = 5.00   # m  — aumentado: cavidade pode ser grande
    CIRCLE_CY_MIN      = 0.10   # m
    CIRCLE_CY_MAX      = 6.00   # m  — aumentado
    CIRCLE_CONFIRM     = 4      # frames (reduzido para detectar mais rápido)

    # ── Centralização ───────────────────────────────────────────────────────
    POCKET_FRONT_TARGET = 0.55  # m — distância ao fundo da cavidade
    POCKET_LAT_TOL      = 0.04  # m
    POCKET_LON_TOL      = 0.04  # m

    def __init__(self):
        super().__init__('wall_follower')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state            = "search"
        self.desired_distance = 0.5

        self.corner_turn_time     = None
        self.CORNER_TURN_DURATION = 1.0

        self.pid_wall    = PID(kp=1.8, ki=0.05, kd=0.35, output_min=-0.45, output_max=0.45)
        self.pid_approach= PID(kp=1.1, ki=0.02, kd=0.18, output_min=-0.40, output_max=0.40)
        self.pid_lat     = PID(kp=1.3, ki=0.00, kd=0.50, output_min=-0.40, output_max=0.40)
        self.pid_lon     = PID(kp=0.7, ki=0.00, kd=0.25, output_min=-0.15, output_max=0.15)

        self.circle_frames = 0
        self._last_circle  = None   # (cx, cy, radius) do último fit válido

        self.get_logger().info("WallFollower v6 — velocidade reduzida")

    # ──────────────────────────────────────── utilidades ────────────────────

    def _normalize(self, deg):
        while deg >  180: deg -= 360
        while deg < -180: deg += 360
        return deg

    def _in_sector(self, angle_deg, lo, hi):
        a, lo, hi = (self._normalize(x) for x in (angle_deg, lo, hi))
        if lo <= hi:
            return lo <= a <= hi
        return a >= lo or a <= hi

    def _sector_dist(self, msg, logical_lo, logical_hi, k=5):
        s_lo = self._normalize(logical_lo + 180)
        s_hi = self._normalize(logical_hi + 180)
        vals = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            a = math.degrees(msg.angle_min + i * msg.angle_increment)
            if self._in_sector(a, s_lo, s_hi):
                vals.append(r)
        if not vals:
            return msg.range_max
        vals.sort()
        return sum(vals[:min(k, len(vals))]) / min(k, len(vals))

    def _get_closest(self, msg):
        best_r, best_a = msg.range_max, None
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < best_r:
                best_r = r
                best_a = msg.angle_min + i * msg.angle_increment
        return best_r, best_a

    # ──────────────────────────── circle fitting ─────────────────────────────

    def _scan_to_cartesian(self, msg, logical_lo, logical_hi):
        points = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.05:
                continue
            # Ângulo do raio no referencial do robô (0=frente, CCW+)
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            if not self._in_sector(angle_deg, logical_lo, logical_hi):
                continue
            # Coordenadas no referencial do robô
            x =  r * math.cos(angle_rad)   # frente
            y =  r * math.sin(angle_rad)   # esquerda
            points.append((x, y))
        return points

    def _detect_circle(self, msg):
        # Só pontos próximos — elimina raios que apontam para espaço aberto
        MAX_RANGE_FOR_FIT = 3.5  # m — ajustar se a cavidade for maior

        points = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.05:
                continue
            if r > MAX_RANGE_FOR_FIT:
                continue  # IGNORA raios que apontam para espaço vazio
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            # Sector completo: frente + esquerda + trás (para capturar
            # toda a parede circular visível de dentro da cavidade)
            if not self._in_sector(angle_deg,
                                   self.CIRCLE_SECTOR_LO,
                                   self.CIRCLE_SECTOR_HI):
                continue
            x = r * math.cos(angle_rad)
            y = r * math.sin(angle_rad)
            points.append((x, y))

        if len(points) < self.CIRCLE_MIN_POINTS:
            self.get_logger().info(
                f"CircleFit: poucos pontos ({len(points)}) após filtro r<{MAX_RANGE_FOR_FIT}"
            )
            return None

        result = fit_circle_lstsq(points)
        if result is None:
            return None

        cx, cy, radius, residual = result

        ok = (
            residual <= self.CIRCLE_MAX_RESIDUAL and
            self.CIRCLE_MIN_RADIUS <= radius <= self.CIRCLE_MAX_RADIUS and
            cy >= self.CIRCLE_CY_MIN and
            cy <= self.CIRCLE_CY_MAX
        )

        # LOG INFO (não debug) para ver sempre o que está a acontecer
        self.get_logger().info(
            f"CircleFit: n={len(points)} cx={cx:.2f} cy={cy:.2f} "
            f"r={radius:.2f} res={residual:.4f} ok={ok}"
        )

        if ok:
            return cx, cy, radius
        return None

    # ─────────────────────────────────────────── callback principal ──────────

    def scan_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9

        closest_dist, closest_angle = self._get_closest(msg)

        front       = self._sector_dist(msg,  -15,   15)
        front_left  = self._sector_dist(msg,   20,   60)
        front_right = self._sector_dist(msg,  -60,  -20)
        left        = self._sector_dist(msg,   75,  105)
        back_left   = self._sector_dist(msg,  120,  160)
        right       = self._sector_dist(msg, -105,  -75)

        closest_deg = math.degrees(closest_angle) if closest_angle is not None else 0.0
        is_corner   = (closest_dist < 0.30 and abs(closest_deg) > 90)

        cmd = Twist()

        # ══ SEARCH ═══════════════════════════════════════════════════════════
        if self.state == "search":
            if closest_angle is None:
                cmd.angular.z = 0.35
            else:
                self.state = "approach"
                self.pid_approach.reset()
                self.get_logger().info("→ APPROACH")

        # ══ APPROACH ═════════════════════════════════════════════════════════
        elif self.state == "approach":
            if closest_angle is None:
                self.state = "search"; return
            if closest_dist <= self.desired_distance:
                self.state = "follow"
                self.pid_wall.reset()
                self.circle_frames = 0
                self.get_logger().info("→ FOLLOW")
            else:
                cmd.linear.x  = 0.12
                cmd.angular.z = self.pid_approach.compute(closest_angle, now)

        # ══ CORNER ═══════════════════════════════════════════════════════════
        elif self.state == "corner":
            if (now - self.corner_turn_time) < self.CORNER_TURN_DURATION:
                cmd.linear.x  = -0.04
                cmd.angular.z = -0.70
            else:
                self.state = "follow"
                self.pid_wall.reset()
                self.circle_frames = 0
                self.get_logger().info("Quina resolvida → FOLLOW")

        # ══ FOLLOW ═══════════════════════════════════════════════════════════
        elif self.state == "follow":

            # ── Detecção de cavidade circular via circle-fitting ──────────
            circle = self._detect_circle(msg)

            if circle is not None:
                cx, cy, radius = circle
                self._last_circle = circle
                self.circle_frames += 1
                self.get_logger().info(
                    f"[CircleFit {self.circle_frames}/{self.CIRCLE_CONFIRM}] "
                    f"cx={cx:.2f} cy={cy:.2f} r={radius:.2f}"
                )
                if self.circle_frames >= self.CIRCLE_CONFIRM:
                    self.state = "center_in_pocket"
                    self.pid_lat.reset()
                    self.pid_lon.reset()
                    self.circle_frames = 0
                    self.get_logger().info(
                        f"★ Cavidade confirmada (r={radius:.2f}m) → CENTER_IN_POCKET"
                    )
                    return
            else:
                self.circle_frames = 0
                self._last_circle  = None

            # ── Manobras de segurança ────────────────────────────────────
            FRONT_BLOCK = 1.0
            LEFT_DANGER = 0.28
            LEFT_WARN   = 0.50

            if is_corner:
                self.state = "corner"
                self.corner_turn_time = now
                self.pid_wall.reset()
                self.circle_frames = 0
                cmd.linear.x  = -0.04
                cmd.angular.z = -0.70
                self.get_logger().info(f"QUINA → CORNER dist={closest_dist:.2f}")

            elif front < FRONT_BLOCK:
                cmd.linear.x  =  0.03
                cmd.angular.z = -0.50
                self.pid_wall.reset()
                self.circle_frames = 0

            elif front_left < FRONT_BLOCK:
                cmd.linear.x  =  0.05
                cmd.angular.z = -0.40
                self.pid_wall.reset()
                self.circle_frames = 0

            elif left < LEFT_DANGER:
                cmd.linear.x  =  0.03
                cmd.angular.z = -0.50
                self.pid_wall.reset()
                self.circle_frames = 0

            elif left > 1.50:
                cmd.linear.x  =  0.08
                cmd.angular.z =  0.25
                self.pid_wall.reset()
                self.circle_frames = 0

            else:
                error = self.desired_distance - left
                if front_left < LEFT_WARN * 2:
                    error += (LEFT_WARN * 2 - front_left) * 0.8
                convex_curve = back_left - front_left
                if convex_curve > 0.3 and front_left < 2.0:
                    error -= convex_curve * 0.5   # empurra para a esquerda

                cmd.linear.x  =  0.10
                cmd.angular.z = self.pid_wall.compute(error, now)

        # ══ CENTER_IN_POCKET ══════════════════════════════════════════════════
        elif self.state == "center_in_pocket":
            # Verifica se ainda está na cavidade
            circle = self._detect_circle(msg)
            if circle is None and left > 1.0:
                self.state = "follow"
                self.pid_wall.reset()
                self.pid_lat.reset()
                self.pid_lon.reset()
                self.circle_frames = 0
                self.get_logger().info("Saiu da cavidade → FOLLOW")
                return

            lateral_error = front_left - front_right
            long_error    = front - self.POCKET_FRONT_TARGET

            lat_ok = abs(lateral_error) < self.POCKET_LAT_TOL
            lon_ok = abs(long_error)    < self.POCKET_LON_TOL

            if lat_ok and lon_ok:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"✓ CENTRADO | fl={front_left:.2f} fr={front_right:.2f} "
                    f"front={front:.2f}"
                )

            elif not lat_ok:
                cmd.angular.z = self.pid_lat.compute(lateral_error, now)
                cmd.linear.x  = 0.0
                self.pid_lon.reset()
                self.get_logger().info(
                    f"LAT | fl={front_left:.2f} fr={front_right:.2f} "
                    f"err={lateral_error:.3f} → ang={cmd.angular.z:.3f}"
                )

            else:
                cmd.linear.x  = self.pid_lon.compute(long_error, now)
                cmd.angular.z = 0.0
                self.pid_lat.reset()
                self.get_logger().info(
                    f"LON | front={front:.2f} target={self.POCKET_FRONT_TARGET:.2f} "
                    f"err={long_error:.3f} → lin={cmd.linear.x:.3f}"
                )

        # ── log geral ─────────────────────────────────────────────────────────
        self.get_logger().info(
            f"[{self.state}] f={front:.2f} fl={front_left:.2f} fr={front_right:.2f} "
            f"l={left:.2f} bl={back_left:.2f} r={right:.2f} "
            f"circle_frames={self.circle_frames} close={closest_dist:.2f}@{closest_deg:.0f}°"
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