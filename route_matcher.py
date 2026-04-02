#!/usr/bin/env python3
import csv
import math
from pathlib import Path
from typing import List, Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(x: float, x_min: float, x_max: float) -> float:
    return max(x_min, min(x_max, x))


def latlon_to_local_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    r = 6378137.0
    x = math.radians(lon - lon0) * r * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * r
    return x, y


class TeachRepeatDualGPSNearestToEnd(Node):
    def __init__(self) -> None:
        super().__init__('teach_repeat_dual_gps_nearest_to_end')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('front_topic', '/gps/front/fix')
        self.declare_parameter('rear_topic', '/gps/rear/fix')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('control_period', 0.1)

        self.declare_parameter('lookahead_points', 5)
        self.declare_parameter('search_forward_window', 20)

        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 0.8)

        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_heading', 2.0)
        self.declare_parameter('k_heading_track', 1.2)
        self.declare_parameter('k_cross_track', 1.5)

        self.declare_parameter('join_reached_distance_m', 0.5)
        self.declare_parameter('final_goal_distance_m', 0.5)
        self.declare_parameter('heading_stop_threshold_rad', 1.0)
        self.declare_parameter('heading_weight_m_per_rad', 0.8)

        self.csv_path_param = str(self.get_parameter('csv_path').value)
        self.front_topic = str(self.get_parameter('front_topic').value)
        self.rear_topic = str(self.get_parameter('rear_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.control_period = float(self.get_parameter('control_period').value)

        self.lookahead_points = int(self.get_parameter('lookahead_points').value)
        self.search_forward_window = int(self.get_parameter('search_forward_window').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.k_linear = float(self.get_parameter('k_linear').value)
        self.k_heading = float(self.get_parameter('k_heading').value)
        self.k_heading_track = float(self.get_parameter('k_heading_track').value)
        self.k_cross_track = float(self.get_parameter('k_cross_track').value)

        self.join_reached_distance_m = float(self.get_parameter('join_reached_distance_m').value)
        self.final_goal_distance_m = float(self.get_parameter('final_goal_distance_m').value)
        self.heading_stop_threshold_rad = float(self.get_parameter('heading_stop_threshold_rad').value)
        self.heading_weight_m_per_rad = float(self.get_parameter('heading_weight_m_per_rad').value)

        self.front_msg: Optional[NavSatFix] = None
        self.rear_msg: Optional[NavSatFix] = None

        self.route_points: List[Dict] = []
        self.ref_lat: Optional[float] = None
        self.ref_lon: Optional[float] = None

        self.mode = 'GO_TO_JOIN'
        self.finished = False

        self.nearest_idx: Optional[int] = None
        self.join_idx: Optional[int] = None
        self.progress_idx: int = 0

        self.last_status: Optional[str] = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.create_subscription(NavSatFix, self.front_topic, self.front_callback, 10)
        self.create_subscription(NavSatFix, self.rear_topic, self.rear_callback, 10)

        csv_path = self.resolve_csv_path()
        self.load_route(csv_path)

        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(f'Using CSV: {csv_path}')
        self.get_logger().info(f'Loaded {len(self.route_points)} route points')
        self.get_logger().info(f'Publishing to: {self.cmd_topic}')

    def front_callback(self, msg: NavSatFix) -> None:
        self.front_msg = msg

    def rear_callback(self, msg: NavSatFix) -> None:
        self.rear_msg = msg

    def resolve_csv_path(self) -> str:
        if self.csv_path_param.strip():
            path = Path(self.csv_path_param).expanduser()
            if not path.exists():
                raise FileNotFoundError(f'CSV file not found: {path}')
            return str(path)

        log_dir = Path.home() / 'log'
        if not log_dir.exists():
            raise FileNotFoundError(f'Log directory not found: {log_dir}')

        candidates = sorted(
            log_dir.glob('teach_path_*.csv'),
            key=lambda p: p.stat().st_mtime,
            reverse=True
        )
        if not candidates:
            raise FileNotFoundError(f'No teach_path_*.csv found in {log_dir}')

        return str(candidates[0])

    def load_route(self, csv_path: str) -> None:
        raw_points: List[Dict] = []
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    raw_points.append({
                        'idx': int(row.get('idx', len(raw_points))),
                        'center_latitude': float(row['center_latitude']),
                        'center_longitude': float(row['center_longitude']),
                        'heading_rad': float(row['heading_rad']),
                    })
                except Exception as e:
                    self.get_logger().warn(f'Skipping bad row: {e}')

        if not raw_points:
            raise RuntimeError('No valid route points loaded')

        self.ref_lat = raw_points[0]['center_latitude']
        self.ref_lon = raw_points[0]['center_longitude']

        for p in raw_points:
            x, y = latlon_to_local_xy(
                p['center_latitude'], p['center_longitude'],
                self.ref_lat, self.ref_lon
            )
            p['x'] = x
            p['y'] = y

        self.route_points = raw_points

    def current_center_and_heading(self) -> Optional[Tuple[float, float, float]]:
        if self.front_msg is None or self.rear_msg is None:
            return None
        if self.ref_lat is None or self.ref_lon is None:
            return None

        front = self.front_msg
        rear = self.rear_msg

        fx, fy = latlon_to_local_xy(front.latitude, front.longitude, self.ref_lat, self.ref_lon)
        rx, ry = latlon_to_local_xy(rear.latitude, rear.longitude, self.ref_lat, self.ref_lon)

        dx = fx - rx
        dy = fy - ry
        baseline = math.hypot(dx, dy)
        if baseline < 1e-6:
            return None

        heading = math.atan2(dy, dx)
        cx = 0.5 * (fx + rx)
        cy = 0.5 * (fy + ry)

        return cx, cy, heading

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def log_status(self, text: str) -> None:
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text

    def find_nearest_index_with_heading(self, x: float, y: float, heading: float) -> int:
        best_idx = 0
        best_score = float('inf')

        for i, p in enumerate(self.route_points):
            dist = math.hypot(x - p['x'], y - p['y'])
            dh = abs(wrap_to_pi(heading - p['heading_rad']))
            score = dist + self.heading_weight_m_per_rad * dh
            if score < best_score:
                best_score = score
                best_idx = i

        return best_idx

    def find_nearest_forward_index(self, x: float, y: float, start_idx: int, window: int) -> int:
        n = len(self.route_points)
        start = max(0, start_idx)
        end = min(n, start + window)

        best_idx = start
        best_dist = float('inf')
        for i in range(start, end):
            p = self.route_points[i]
            d = math.hypot(x - p['x'], y - p['y'])
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def point_to_segment_cross_track(
        self, x: float, y: float, ax: float, ay: float, bx: float, by: float
    ) -> float:
        vx = bx - ax
        vy = by - ay
        wx = x - ax
        wy = y - ay

        seg_len2 = vx * vx + vy * vy
        if seg_len2 < 1e-9:
            return 0.0

        t = (wx * vx + wy * vy) / seg_len2
        t = clamp(t, 0.0, 1.0)

        proj_x = ax + t * vx
        proj_y = ay + t * vy

        dx = x - proj_x
        dy = y - proj_y
        dist = math.hypot(dx, dy)

        cross = vx * (y - ay) - vy * (x - ax)
        sign = 1.0 if cross >= 0.0 else -1.0
        return sign * dist

    def compute_go_to_point_cmd(
        self,
        current_x: float,
        current_y: float,
        current_heading: float,
        target_x: float,
        target_y: float
    ) -> Tuple[Twist, float, float]:
        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)

        target_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_heading - current_heading)

        cmd = Twist()
        cmd.angular.z = clamp(self.k_heading * heading_error, -self.max_angular_speed, self.max_angular_speed)

        if abs(heading_error) > self.heading_stop_threshold_rad:
            cmd.linear.x = 0.0
        else:
            v = clamp(self.k_linear * dist, 0.0, self.max_linear_speed)
            v *= max(0.0, math.cos(heading_error))
            cmd.linear.x = v

        return cmd, dist, heading_error

    def compute_follow_cmd(
        self,
        current_x: float,
        current_y: float,
        current_heading: float,
        base_idx: int
    ) -> Tuple[Twist, int, float, float]:
        target_idx = min(base_idx + self.lookahead_points, len(self.route_points) - 1)
        target = self.route_points[target_idx]

        dx = target['x'] - current_x
        dy = target['y'] - current_y
        dist_to_target = math.hypot(dx, dy)

        heading_to_target = math.atan2(dy, dx)
        heading_error = wrap_to_pi(heading_to_target - current_heading)

        route_heading = self.route_points[base_idx]['heading_rad']
        heading_track_error = wrap_to_pi(route_heading - current_heading)

        if base_idx < len(self.route_points) - 1:
            p0 = self.route_points[base_idx]
            p1 = self.route_points[base_idx + 1]
            cross_track = self.point_to_segment_cross_track(
                current_x, current_y,
                p0['x'], p0['y'],
                p1['x'], p1['y']
            )
        else:
            cross_track = 0.0

        cmd = Twist()
        angular = (
            self.k_heading * heading_error
            + self.k_heading_track * heading_track_error
            + self.k_cross_track * math.atan2(cross_track, 1.0)
        )
        cmd.angular.z = clamp(angular, -self.max_angular_speed, self.max_angular_speed)

        if abs(heading_error) > self.heading_stop_threshold_rad:
            cmd.linear.x = 0.0
        else:
            v = self.max_linear_speed
            v *= max(0.15, math.cos(heading_error))
            if abs(cross_track) > 0.6:
                v *= 0.5
            elif abs(cross_track) > 0.3:
                v *= 0.75
            if dist_to_target < 0.5:
                v *= 0.7
            cmd.linear.x = clamp(v, 0.0, self.max_linear_speed)

        return cmd, target_idx, abs(cross_track), dist_to_target

    def control_loop(self) -> None:
        if self.finished:
            self.publish_stop()
            return

        state = self.current_center_and_heading()
        if state is None:
            self.log_status('Waiting for /gps/front/fix and /gps/rear/fix ...')
            return

        current_x, current_y, current_heading = state

        if self.mode == 'GO_TO_JOIN':
            self.nearest_idx = self.find_nearest_index_with_heading(current_x, current_y, current_heading)
            self.join_idx = self.nearest_idx
            self.progress_idx = self.join_idx

            target = self.route_points[self.join_idx]
            cmd, dist, heading_error = self.compute_go_to_point_cmd(
                current_x, current_y, current_heading,
                target['x'], target['y']
            )
            self.cmd_pub.publish(cmd)

            self.log_status(
                f'[GO_TO_JOIN] nearest_idx={self.nearest_idx} dist={dist:.3f}m heading_err={heading_error:.3f}'
            )

            if dist <= self.join_reached_distance_m:
                self.mode = 'FOLLOW_ROUTE'
                self.progress_idx = self.join_idx
                self.log_status(f'Switched to FOLLOW_ROUTE at idx={self.progress_idx}')

        elif self.mode == 'FOLLOW_ROUTE':
            nearest_forward = self.find_nearest_forward_index(
                current_x, current_y, self.progress_idx, self.search_forward_window
            )
            self.progress_idx = max(self.progress_idx, nearest_forward)

            cmd, target_idx, cross_track_err, dist_to_target = self.compute_follow_cmd(
                current_x, current_y, current_heading, self.progress_idx
            )
            self.cmd_pub.publish(cmd)

            self.log_status(
                f'[FOLLOW_ROUTE] progress_idx={self.progress_idx} target_idx={target_idx} '
                f'cross_track={cross_track_err:.3f}m dist_target={dist_to_target:.3f}m'
            )

            if target_idx >= len(self.route_points) - 1 and dist_to_target <= self.final_goal_distance_m:
                self.get_logger().info('Reached final route point. Stopping.')
                self.finished = True
                self.publish_stop()

        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')
            self.publish_stop()

    def destroy_node(self):
        try:
            if rclpy.ok():
                self.publish_stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeachRepeatDualGPSNearestToEnd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
