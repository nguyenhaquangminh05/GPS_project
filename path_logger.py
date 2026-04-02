#!/usr/bin/env python3
import csv
import math
from pathlib import Path
from datetime import datetime
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r * c


def latlon_to_local_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    r = 6378137.0
    x = math.radians(lon - lon0) * r * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * r
    return x, y


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TeachLoggerDualGPS(Node):
    def __init__(self) -> None:
        super().__init__('teach_logger_dual_gps')

        self.declare_parameter('front_topic', '/gps/front/fix')
        self.declare_parameter('rear_topic', '/gps/rear/fix')
        self.declare_parameter('log_dir', str(Path.home() / 'log'))
        self.declare_parameter('timer_period', 0.2)
        self.declare_parameter('min_center_distance_m', 0.20)
        self.declare_parameter('min_heading_change_rad', 0.10)

        self.front_topic = str(self.get_parameter('front_topic').value)
        self.rear_topic = str(self.get_parameter('rear_topic').value)
        self.log_dir = Path(str(self.get_parameter('log_dir').value)).expanduser()
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.min_center_distance_m = float(self.get_parameter('min_center_distance_m').value)
        self.min_heading_change_rad = float(self.get_parameter('min_heading_change_rad').value)

        self.front_msg: Optional[NavSatFix] = None
        self.rear_msg: Optional[NavSatFix] = None

        self.ref_lat: Optional[float] = None
        self.ref_lon: Optional[float] = None

        self.prev_logged_lat: Optional[float] = None
        self.prev_logged_lon: Optional[float] = None
        self.prev_logged_heading: Optional[float] = None

        self.sample_count = 0

        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_path = self.log_dir / f'teach_path_{timestamp}.csv'

        self.csv_file = open(self.output_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'idx',
            't_sec',
            'front_stamp_sec',
            'front_stamp_nsec',
            'rear_stamp_sec',
            'rear_stamp_nsec',
            'front_latitude',
            'front_longitude',
            'rear_latitude',
            'rear_longitude',
            'center_latitude',
            'center_longitude',
            'heading_rad',
            'heading_deg',
            'baseline_m',
        ])
        self.csv_file.flush()

        self.create_subscription(NavSatFix, self.front_topic, self.front_callback, 10)
        self.create_subscription(NavSatFix, self.rear_topic, self.rear_callback, 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f'Logging to: {self.output_path}')
        self.get_logger().info(f'Front GPS: {self.front_topic}')
        self.get_logger().info(f'Rear GPS: {self.rear_topic}')

    def front_callback(self, msg: NavSatFix) -> None:
        self.front_msg = msg
        if self.ref_lat is None or self.ref_lon is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude

    def rear_callback(self, msg: NavSatFix) -> None:
        self.rear_msg = msg
        if self.ref_lat is None or self.ref_lon is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude

    def compute_center_and_heading(self) -> Optional[Tuple[float, float, float, float]]:
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

        center_lat = 0.5 * (front.latitude + rear.latitude)
        center_lon = 0.5 * (front.longitude + rear.longitude)

        return center_lat, center_lon, heading, baseline

    def should_log(self, center_lat: float, center_lon: float, heading: float) -> bool:
        if self.prev_logged_lat is None or self.prev_logged_lon is None or self.prev_logged_heading is None:
            return True

        dist = haversine_distance_m(
            self.prev_logged_lat, self.prev_logged_lon,
            center_lat, center_lon
        )
        dheading = abs(wrap_to_pi(heading - self.prev_logged_heading))

        return (dist >= self.min_center_distance_m) or (dheading >= self.min_heading_change_rad)

    def timer_callback(self) -> None:
        result = self.compute_center_and_heading()
        if result is None:
            return

        center_lat, center_lon, heading, baseline = result
        if not self.should_log(center_lat, center_lon, heading):
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        front = self.front_msg
        rear = self.rear_msg
        assert front is not None and rear is not None

        self.writer.writerow([
            self.sample_count,
            f'{now_sec:.6f}',
            int(front.header.stamp.sec),
            int(front.header.stamp.nanosec),
            int(rear.header.stamp.sec),
            int(rear.header.stamp.nanosec),
            f'{front.latitude:.12f}',
            f'{front.longitude:.12f}',
            f'{rear.latitude:.12f}',
            f'{rear.longitude:.12f}',
            f'{center_lat:.12f}',
            f'{center_lon:.12f}',
            f'{heading:.6f}',
            f'{math.degrees(heading):.3f}',
            f'{baseline:.6f}',
        ])
        self.csv_file.flush()

        self.prev_logged_lat = center_lat
        self.prev_logged_lon = center_lon
        self.prev_logged_heading = heading
        self.sample_count += 1

        if self.sample_count % 10 == 0:
            self.get_logger().info(f'Logged {self.sample_count} points')

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f'Saved to: {self.output_path}')
            self.get_logger().info(f'Total points: {self.sample_count}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeachLoggerDualGPS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
