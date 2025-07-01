import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix


class TFMatrixDemo(Node):
    def __init__(self):
        super().__init__('tf_matrix_demo')

        # 初始化TF2监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 设置源坐标系和目标坐标系
        self.source_frame = 'base_link'
        self.target_frame = 'odom'

        # 创建定时器，定期获取TF转换
        self.timer = self.create_timer(1.0, self.get_transform_and_convert)
        self.get_logger().info(f"TF矩阵演示节点已启动，监听 {self.source_frame} 到 {self.target_frame} 的转换")

    def get_transform_and_convert(self):
        try:
            # 获取TF转换
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())

            self.process_transform(transform_stamped)

        except TransformException as ex:
            self.get_logger().error(f"获取TF转换失败: {ex}")

    def process_transform(self, transform_stamped):
        """处理TF转换，转换为矩阵并还原参数"""
        # 提取平移和旋转
        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation

        # 打印原始TF数据
        self.get_logger().info(f"原始TF转换:")
        self.get_logger().info(f"  平移: x={translation.x}, y={translation.y}, z={translation.z}")
        self.get_logger().info(f"  旋转: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

        # 创建平移向量
        trans_vector = np.array([translation.x, translation.y, translation.z])

        # 创建旋转四元数
        quat = (rotation.x, rotation.y, rotation.z, rotation.w)

        # 从四元数创建旋转矩阵（使用tf_transformations）
        rot_matrix = quaternion_matrix(quat)[:3, :3]

        # 创建4x4变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rot_matrix
        transform_matrix[:3, 3] = trans_vector

        # 打印变换矩阵
        self.get_logger().info("变换矩阵 (4x4):")
        self.get_logger().info(str(transform_matrix))

        # 从矩阵还原为7个参数
        # 提取平移部分
        recovered_trans = transform_matrix[:3, 3]
        # 从矩阵提取旋转四元数（使用tf_transformations）
        recovered_quat = quaternion_from_matrix(transform_matrix)

        # 打印还原的7个参数 (x,y,z,qx,qy,qz,qw)
        self.get_logger().info("从矩阵还原的7个参数:")
        self.get_logger().info(f"  平移: x={recovered_trans[0]:.6f}, y={recovered_trans[1]:.6f}, z={recovered_trans[2]:.6f}")
        self.get_logger().info(f"  四元数: x={recovered_quat[0]:.6f}, y={recovered_quat[1]:.6f}, z={recovered_quat[2]:.6f}, w={recovered_quat[3]:.6f}")
        self.get_logger().info("---")


def main(args=None):
    rclpy.init(args=args)
    node = TFMatrixDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()