#!/usr/bin/env python3
# 给真实机械臂下发moveit规划出的轨迹数据

from motors.feetech import FeetechMotorsBus
import json
import time
import os
import argparse

def load_trajectory(filepath):
    with open(filepath, 'r') as f:
        return json.load(f)

def rad_to_steps(rad, scale=1.0):
    """将弧度转换为电机步数
    Args:
        rad: 弧度值
        scale: 缩放比例（用于缩小运动范围）
    Returns:
        电机步数（0-4096）
    """
    # 将弧度缩小指定比例
    rad = rad / scale
    # 转换为角度
    degree = rad * 180.0 / 3.14159
    # 转换为步数（2048对应0度，4096步对应360度）
    steps = int(2048 + (degree * 4096 / 360.0))
    # 确保步数在有效范围内
    steps = max(min(steps, 4095), 0)
    return steps

def main():
    parser = argparse.ArgumentParser(description='Execute a recorded trajectory on SO-ARM100')
    parser.add_argument('trajectory_file', help='Path to the trajectory JSON file')
    args = parser.parse_args()

    # 定义电机配置
    motors = {
        "shoulder_pan_joint": (1, "sts3215"),
        "shoulder_lift_joint": (2, "sts3215"),
        "elbow_joint": (3, "sts3215"),
        "wrist_pitch_joint": (4, "sts3215"),
        "wrist_roll_joint": (5, "sts3215"),
        "jaw_joint": (6, "sts3215")
    }

    try:
        # 加载轨迹
        trajectory_data = load_trajectory(args.trajectory_file)
        print(f"Loaded trajectory with {len(trajectory_data['points'])} points")

        # 初始化总线
        bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
        print("Connecting to motors bus...")
        bus.connect()

        # 等待电机准备就绪
        time.sleep(1)

        # 启用扭矩并设置P系数
        print("\nEnabling torque and setting P coefficient...")
        bus.write("Torque_Enable", [1] * len(motors))
        time.sleep(0.5)
        bus.write("P_Coefficient", [32] * len(motors))
        time.sleep(0.5)

        # 执行轨迹中的每个点
        for i, point in enumerate(trajectory_data['points']):
            print(f"\nExecuting point {i+1}/{len(trajectory_data['points'])}")
            
            # 将位置从弧度转换为电机步数，并应用1/50的缩放
            for joint_name, position in zip(trajectory_data['joint_names'], point['positions']):
                motor_id = motors[joint_name][0]
                steps = rad_to_steps(position, scale=1.0)
                print(f"Setting {joint_name} (ID: {motor_id}) to {steps} steps")
                bus.write("Goal_Position", steps, joint_name)
            
            # 等待移动完成
            time.sleep(point['time_from_start'] if i < len(trajectory_data['points'])-1 
                      else 1.0)  # 最后一个点多等待一秒

        print("\nTrajectory execution completed")

    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'bus' in locals():
            bus.disconnect()

if __name__ == "__main__":
    main()
