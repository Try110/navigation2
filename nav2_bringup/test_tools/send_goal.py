import typing

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Pose
from loguru import logger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class MyPose:
    def __init__(self, name: str = '', x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw

    def to_geo_pose(self) -> Pose:
        temp_pose: Pose = Pose()
        temp_pose.position.x = float(self.x)
        temp_pose.position.y = float(self.y)
        temp_pose.position.z = float(self.z)
        temp_pose.orientation.x = float(self.qx)
        temp_pose.orientation.y = float(self.qy)
        temp_pose.orientation.z = float(self.qz)
        temp_pose.orientation.w = float(self.qw)
        return temp_pose

    def __str__(self):
        return 'name: %s' % self.name


def read_poses_from_yaml(file_path):
    """从YAML文件读取所有pose数据"""
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    if 'poses' not in data:
        logger.info("错误: YAML文件中未找到'poses'字段")
        return []
    poses = []
    for pose_name, pose_data in data['poses'].items():
        name = pose_data.get('name', '')
        position = pose_data.get('position', {})
        orientation = pose_data.get('orientation', {})
        poses.append(MyPose(name, position.get('x', 0.0), position.get('y', 0.0), position.get('z', 0.0),
                            orientation.get('x', 0.0), orientation.get('y', 0.0), orientation.get('z', 0.0),
                            orientation.get('w', 1.0)))
    return poses


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('goal_sender_node')
        self.goal_sender_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.pose_list: typing.List[MyPose] = read_poses_from_yaml('poses.yaml')
        self.pose_index: int = 1
        self.send_goal()

    def send_goal(self):
        def _send_goal(_pose: MyPose):
            self.get_logger().info('Sending goal: {0}'.format(_pose))
            goal_msg: NavigateToPose.Goal = NavigateToPose.Goal()
            goal_msg.behavior_tree = ''
            temp_pose = PoseStamped()
            temp_pose.header.frame_id = 'map'
            temp_pose.header.stamp = self.get_clock().now().to_msg()
            temp_pose.pose = _pose.to_geo_pose()
            goal_msg.pose = temp_pose

            self.goal_sender_client.wait_for_server()

            self._send_goal_future = self.goal_sender_client.send_goal_async(goal_msg,
                                                                             feedback_callback=self.feedback_callback)

            self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.pose_index += 1
        goal_pose: MyPose = self.pose_list[self.pose_index % len(self.pose_list)]
        _send_goal(goal_pose)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self._timer = self.create_timer()
        self.send_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
