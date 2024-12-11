#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
import sys
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
import trajectory_msgs.msg
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        # 创建服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')
            
        self.trajectory_display_publisher = self.create_publisher(
            DisplayTrajectory,
            '/display_planned_path',
            10)
            
        self.get_logger().info('Robot Mover initialized')
        
        # 创建动作客户端
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 等待动作服务器启动
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
            
        self.get_logger().info('Action client ready')
        
    def move_arm(self, joint_positions):
        """移动机械臂到指定关节位置"""
        self.get_logger().info('Moving arm to target joint positions')
        
        # 创建并发送轨迹目标
        goal_msg = FollowJointTrajectory.Goal()
        
        # 设置关节名称
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]
        
        # 创建轨迹点
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=2, nanosec=0).to_msg()
        
        goal_msg.trajectory.points.append(point)
        
        # 发送目标并等待结果
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
            
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

def main():
    rclpy.init()
    robot_mover = RobotMover()
    
    try:
        # 示例：移动到特定关节角度
        joint_positions = [0.0, -1.0, 1.0, 0.0, 0.0]  # 对应五个关节
        robot_mover.move_arm(joint_positions)
        
        executor = MultiThreadedExecutor()
        executor.add_node(robot_mover)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            executor.shutdown()
            robot_mover.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
