#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class FakeGpsPublisher(Node):
    def __init__(self):
        super().__init__('fake_gps_publisher')

        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.publish_fake_gps)

        self.base_lat = 21.028511
        self.base_lon = 105.852020
        self.base_alt = 5.0

        self.lat_range = 0.0008
        self.lon_range = 0.0008

        self.get_logger().info('Publishing fake GPS on /gps/fix')

    def publish_fake_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        msg.latitude = self.base_lat + random.uniform(-self.lat_range, self.lat_range)
        msg.longitude = self.base_lon + random.uniform(-self.lon_range, self.lon_range)
        msg.altitude = self.base_alt + random.uniform(-1.0, 1.0)

        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = 0

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, alt={msg.altitude:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeGpsPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
