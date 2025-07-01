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
        # 初始化TF2监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_init_pose, 10)
        self.pose: Pose = Pose()
        self.timer = self.create_timer(0.02, lambda: self.publish_tf('map', 'odom'))  # 50Hz (1/50 = 0.02s)
        self.get_logger().info("Publishing map->odom TF at 50Hz...")

        # self.timer_1 = self.create_timer(10, lambda: self.on_init_pose(self.get_test_pose()))  # 50Hz (1/50 = 0.02s)



    def get_test_pose(self):
        transform_stamped = self.get_transform(target_frame='map', source_frame='base_link')
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = transform_stamped.transform.translation.x
        pose.pose.pose.position.y = transform_stamped.transform.translation.y
        pose.pose.pose.position.z = transform_stamped.transform.translation.z

        pose.pose.pose.orientation.x = transform_stamped.transform.rotation.x
        pose.pose.pose.orientation.y = transform_stamped.transform.rotation.y
        pose.pose.pose.orientation.z = transform_stamped.transform.rotation.z
        pose.pose.pose.orientation.w = transform_stamped.transform.rotation.w
        return pose

    def get_transform(self, target_frame: str, source_frame: str) -> TransformStamped:
        transform_stamped = TransformStamped()
        try:
            # 获取TF转换
            transform_stamped: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
            return transform_stamped
        except TransformException as ex:
            self.get_logger().error(f"获取TF转换失败: {ex}")
        finally:
            return transform_stamped

    def on_init_pose(self, msg: PoseWithCovarianceStamped):
        def to_mat(translation, rotation) -> np.ndarray:
            trans_vector = np.array([translation.x, translation.y, translation.z])
            # 创建旋转四元数
            quat = (rotation.x, rotation.y, rotation.z, rotation.w)
            # 从四元数创建旋转矩阵（使用tf_transformations）
            rot_matrix = quaternion_matrix(quat)[:3, :3]
            # 创建4x4变换矩阵
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rot_matrix
            transform_matrix[:3, 3] = trans_vector
            return transform_matrix

        def to_trans_quat(transform_matrix):
            recovered_trans = transform_matrix[:3, 3]
            # 从矩阵提取旋转四元数（使用tf_transformations）
            recovered_quat = quaternion_from_matrix(transform_matrix)
            return recovered_trans, recovered_quat

        # self.pose = msg.pose.pose
        base_link2map_transform_matrix_ = to_mat(msg.pose.pose.position, msg.pose.pose.orientation)
        base_link2odom_transform_stamped: TransformStamped = self.get_transform(target_frame='odom',
                                                                                source_frame='base_link')
        base_link2odom_transform_matrix_ = to_mat(base_link2odom_transform_stamped.transform.translation,
                                                  base_link2odom_transform_stamped.transform.rotation)
        # 计算得来的
        odom2map_matrix_ = np.linalg.inv(base_link2odom_transform_matrix_) @ base_link2map_transform_matrix_
        recovered_trans, recovered_quat = to_trans_quat(odom2map_matrix_)

        odom2map_transform_stamped: TransformStamped = self.get_transform(target_frame='map',
                                                                          source_frame='odom')
        odom2map_transform_matrix_ = to_mat(odom2map_transform_stamped.transform.translation,
                                            odom2map_transform_stamped.transform.rotation)
        pass
        self.pose.position.x = recovered_trans[0]
        self.pose.position.y = recovered_trans[1]
        self.pose.position.z = recovered_trans[2]
        self.pose.orientation.x = recovered_quat[0]
        self.pose.orientation.y = recovered_quat[1]
        self.pose.orientation.z = recovered_quat[2]
        self.pose.orientation.w = recovered_quat[3]

    def publish_tf(self, frame_id: str = 'map', child_frame_id: str = 'odom'):
        t = TransformStamped()
        # 设置TF关系的基本信息
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id  # 父坐标系
        t.child_frame_id = child_frame_id  # 子坐标系

        # 设置变换（假设map和odom坐标系重合）
        t.transform.translation.x = self.pose.position.x
        t.transform.translation.y = self.pose.position.y
        t.transform.translation.z = self.pose.position.z

        # 无旋转（四元数单位元表示无旋转）
        t.transform.rotation.x = self.pose.orientation.x
        t.transform.rotation.y = self.pose.orientation.y
        t.transform.rotation.z = self.pose.orientation.z
        t.transform.rotation.w = self.pose.orientation.w
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
