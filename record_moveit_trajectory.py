#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import json
import os
from datetime import datetime

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        
        self.get_logger().info('Node initialized')
        
        # 监听显示的轨迹
        self.display_trajectory_sub = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.display_trajectory_callback,
            10)
        self.get_logger().info('Created subscription to /display_planned_path')
        
        # 创建轨迹存储目录
        self.trajectories_dir = os.path.join(os.path.dirname(__file__), 'recorded_trajectories')
        os.makedirs(self.trajectories_dir, exist_ok=True)
        self.get_logger().info(f'Created trajectories directory: {self.trajectories_dir}')
        
        self.get_logger().info('Trajectory recorder started. Waiting for trajectories...')
        self.get_logger().info('Please click "Plan" in MoveIt to generate a trajectory.')

    def display_trajectory_callback(self, msg):
        """当收到规划的轨迹时被调用"""
        self.get_logger().info('Received trajectory from /display_planned_path')
        
        if not msg.trajectory:
            self.get_logger().warn('Received empty trajectory message')
            return
            
        # 处理所有轨迹
        for i, trajectory in enumerate(msg.trajectory):
            self.get_logger().info(f'Processing trajectory {i+1}/{len(msg.trajectory)}')
            
            # 获取关节轨迹
            joint_trajectory = trajectory.joint_trajectory
            if not joint_trajectory.points:
                self.get_logger().warn(f'Trajectory {i+1} has no points')
                continue
                
            self.get_logger().info(f'Joint names: {joint_trajectory.joint_names}')
            self.get_logger().info(f'Number of points: {len(joint_trajectory.points)}')
            
            # 保存轨迹
            trajectory_data = {
                'joint_names': joint_trajectory.joint_names,
                'points': []
            }
            
            # 记录每个时间点的关节位置
            for point in joint_trajectory.points:
                point_data = {
                    'positions': list(point.positions),
                    'velocities': list(point.velocities) if point.velocities else [],
                    'accelerations': list(point.accelerations) if point.accelerations else [],
                    'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                }
                trajectory_data['points'].append(point_data)
            
            # 生成文件名（使用时间戳和轨迹索引）
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'trajectory_{timestamp}_part{i+1}.json'
            filepath = os.path.join(self.trajectories_dir, filename)
            
            # 保存轨迹数据到JSON文件
            with open(filepath, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            
            self.get_logger().info(f'Saved trajectory to {filepath}')

def main():
    rclpy.init()
    recorder = TrajectoryRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
