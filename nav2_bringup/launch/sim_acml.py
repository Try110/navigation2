#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException



class MyNode(Node):
    def __init__(self):
        super().__init__('sim_localization_1')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, lambda: self.publish_tf('map', 'odom'))  # 50Hz (1/50 = 0.02s)
        self.get_logger().info("Publishing map->odom TF at 50Hz...")

    def publish_tf(self, frame_id: str = 'map', child_frame_id: str = 'odom'):
        t = TransformStamped()
        # 设置TF关系的基本信息
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id  # 父坐标系
        t.child_frame_id = child_frame_id  # 子坐标系

        # 设置变换（假设map和odom坐标系重合）
        t.transform.translation.x = 9.048036623143718
        t.transform.translation.y = 5.753439726187424
        t.transform.translation.z = -0.004769324124489926

        # 无旋转（四元数单位元表示无旋转）
        t.transform.rotation.x = -4.9313622044192555e-06
        t.transform.rotation.y = -0.0007772914139506648
        t.transform.rotation.z = -0.006010931804569876
        t.transform.rotation.w = 0.9999816320776002
        # self.get_logger().info("send tf from %s -> %s" % (frame_id, child_frame_id))

        # 发布TF
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
