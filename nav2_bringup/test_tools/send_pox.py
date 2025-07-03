#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np


def rotate_point(point, angle):
    """Rotate a 2D point around origin by angle (in radians)"""
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])
    return np.dot(R, point)


class PolygonPublisher(Node):

    def __init__(self):
        super().__init__('polygon_publisher')
        self.publisher_ = self.create_publisher(PolygonStamped, 'visualization_polygon', 10)

        # 原始多边形顶点（局部坐标系下）
        self.polygon_local = [
            [-0.385, 0.0],
            [-0.376, -0.134],
            [-0.331, -0.224],
            [-0.272, -0.269],
            [0.281, -0.266],
            [0.355, -0.203],
            [0.379, -0.104],
            [0.385, 0.0],
            [0.379, 0.125],
            [0.343, 0.212],
            [0.293, 0.26],
            [-0.284, 0.263],
            [-0.331, 0.221],
            [-0.367, 0.158]
        ]

        # 设置目标 pose: x, y, theta (弧度)
        self.pose = (4.20467472076416, 3.565551519393921, np.deg2rad(0))  # 示例：x=1, y=2, yaw=45度

        self.timer = self.create_timer(1.0, self.timer_callback)  # 每秒发送一次

    def timer_callback(self):
        polygon_transformed = []

        x, y, theta = self.pose

        for pt in self.polygon_local:
            # 旋转
            rotated = rotate_point(pt, theta)
            # 平移
            transformed = (rotated[0] + x, rotated[1] + y)
            polygon_transformed.append(transformed)

        # 构造 PolygonStamped 消息
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # 确保与你的坐标系一致，如 map 或 odom

        for p in polygon_transformed:
            point = Point32()
            point.x = float(p[0])
            point.y = float(p[1])
            point.z = 0.0
            msg.polygon.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing polygon.')


def main(args=None):
    rclpy.init(args=args)

    polygon_publisher = PolygonPublisher()

    rclpy.spin(polygon_publisher)

    # Destroy the node explicitly
    polygon_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
