#!/usr/bin/env python3
# 给真实机械臂下发moveit规划出的轨迹数据

from motors.feetech import FeetechMotorsBus
import json
import time
import os
import argparse
import math

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

        # 先检查当前扭矩状态
        print("\nChecking current torque status...")
        for joint_name in motors.keys():
            torque_status = bus.read("Torque_Enable", joint_name)
            print(f"{joint_name}: Torque {'Enabled' if torque_status else 'Disabled'}")

        # 确保所有关节的扭矩都被启用
        print("\nEnabling torque for all joints...")
        for joint_name in motors.keys():
            print(f"Enabling torque for {joint_name}")
            bus.write("Torque_Enable", 1, joint_name)
            time.sleep(0.1)  # 给每个关节一点时间来启用扭矩

        # 再次检查扭矩状态
        print("\nVerifying torque status...")
        for joint_name in motors.keys():
            torque_status = bus.read("Torque_Enable", joint_name)
            print(f"{joint_name}: Torque {'Enabled' if torque_status else 'Disabled'}")
            if not torque_status:
                raise Exception(f"Failed to enable torque for {joint_name}")

        # 设置P系数
        print("\nSetting P coefficients...")
        p_coefficients = {
            "shoulder_pan_joint": 24,    # 底座关节，负载最大，需要最小的P值
            "shoulder_lift_joint": 24,   # 大臂关节，负载大
            "elbow_joint": 28,          # 肘部关节，中等负载
            "wrist_pitch_joint": 32,    # 腕部关节，负载较小
            "wrist_roll_joint": 32,     # 腕部旋转，负载较小
            "jaw_joint": 35             # 夹爪，负载最小
        }
        
        for joint_name, p_value in p_coefficients.items():
            print(f"Setting P coefficient for {joint_name} to {p_value}")
            bus.write("P_Coefficient", p_value, joint_name)
            time.sleep(0.1)

        print("\nStarting trajectory execution...")

        # 执行轨迹中的每个点
        last_positions = None
        for i, point in enumerate(trajectory_data['points']):
            print(f"\nExecuting point {i+1}/{len(trajectory_data['points'])}")
            
            # 第一个点特殊处理：缓慢移动到起始位置
            if i == 0:
                print("Moving to initial position...")
                # 先读取当前位置
                current_positions = []
                for joint_name in trajectory_data['joint_names']:
                    current_pos = bus.read("Present_Position", joint_name)
                    # 确保我们获取数组的第一个元素
                    if hasattr(current_pos, '__len__'):
                        current_pos = current_pos[0]
                    current_rad = (current_pos - 2048) * 360.0 / 4096 * 3.14159 / 180.0
                    current_positions.append(float(current_rad))
                
                # 计算到目标位置的最大角度差
                target_positions = [float(pos) for pos in point['positions']]
                max_diff = max(abs(curr - target) for curr, target in zip(current_positions, target_positions))
                print(f"Distance to initial position: {max_diff:.4f} rad")
                
                # 进一步减少插值点数量以加快速度
                steps = max(20, min(70, int(max_diff / 0.007)))  # 每0.007弧度一个点，最少20点，最多70点
                print(f"Moving to initial position with {steps} interpolation points...")
                
                # 立即开始移动到第一个插值点，以避免初始延迟
                first_positions = [
                    curr + (target - curr) * 0.08  # 移动8%的距离
                    for curr, target in zip(current_positions, target_positions)
                ]
                for joint_name, position in zip(trajectory_data['joint_names'], first_positions):
                    steps = rad_to_steps(position, scale=1.0)
                    bus.write("Goal_Position", steps, joint_name)
                
                # 极短暂等待
                time.sleep(0.015)  # 从0.02减少到0.015
                
                # 继续其余的插值移动
                for step in range(2, steps + 1):
                    # 使用平滑的加速度曲线
                    t = step / steps
                    t = t * t * (3 - 2 * t)  # 仅使用二次平滑
                    
                    # 计算插值位置
                    interp_positions = [
                        curr + (target - curr) * t
                        for curr, target in zip(current_positions, target_positions)
                    ]
                    
                    # 设置所有关节位置
                    for joint_name, position in zip(trajectory_data['joint_names'], interp_positions):
                        steps = rad_to_steps(position, scale=1.0)
                        bus.write("Goal_Position", steps, joint_name)
                    
                    # 进一步缩短等待时间
                    progress = step / steps
                    base_wait = 0.007  # 从0.01减少到0.007
                    if progress < 0.15:  # 加速阶段缩短
                        wait_time = base_wait * (1 + progress * 0.3)
                    elif progress > 0.85:  # 减速阶段缩短
                        wait_time = base_wait * (1 + (1 - progress) * 0.3)
                    else:  # 匀速阶段
                        wait_time = base_wait
                    
                    time.sleep(wait_time)
                    
                    # 只在50%时打印进度
                    if step == steps // 2:
                        print(f"Initial movement progress: 50%")
                
                print("Initial movement completed, waiting for stabilization...")
                # 缩短稳定时间
                time.sleep(0.7)  # 从1.0减少到0.7
                last_positions = point['positions']
                continue

            # 计算当前点与上一个点的最大位置差异
            max_diff = None
            if last_positions is not None:
                max_diff = max(abs(a - b) for a, b in zip(point['positions'], last_positions))
                print(f"Max position difference: {max_diff:.4f} rad")
                
                # 始终进行插值，但根据差异大小调整插值点数量
                base_steps = max(5, min(30, int(max_diff / 0.008)))  # 增加基础插值点数量
                print(f"Interpolating with {base_steps} points...")
                
                # 使用更平滑的插值
                for step in range(1, base_steps + 1):
                    # 使用改进的平滑函数：组合余弦和二次函数
                    t = step / base_steps
                    # 在开始和结束时更平滑
                    t = 0.5 * (1 - math.cos(math.pi * t))
                    t = t * t * (3 - 2 * t)  # 额外的平滑
                    
                    interp_positions = [
                        last + (curr - last) * t
                        for last, curr in zip(last_positions, point['positions'])
                    ]
                    
                    # 同时设置所有关节位置
                    for joint_name, position in zip(trajectory_data['joint_names'], interp_positions):
                        steps = rad_to_steps(position, scale=1.0)
                        bus.write("Goal_Position", steps, joint_name)
                    
                    # 更细致的动态等待时间
                    if max_diff > 0.1:  # 大运动
                        wait_time = max(0.03, min(0.06, max_diff * 0.15))
                    else:  # 小运动
                        wait_time = max(0.02, min(0.04, max_diff * 0.1))
                    time.sleep(wait_time)
            
            # 记录当前位置用于下一次比较
            last_positions = point['positions']
            
            # 轨迹点之间的等待时间
            if i < len(trajectory_data['points']) - 1:
                next_time = trajectory_data['points'][i + 1]['time_from_start']
                current_time = point['time_from_start']
                planned_wait_time = next_time - current_time
                
                # 根据运动大小调整等待时间
                if max_diff is not None:
                    if max_diff > 0.1:  # 大运动多等待一点
                        wait_time = max(0.08, min(0.2, max_diff * 0.4))
                    else:  # 小运动等待短一些
                        wait_time = max(0.05, min(0.15, max_diff * 0.3))
                else:
                    wait_time = 0.1
                
                print(f"Waiting for {wait_time:.2f} seconds...")
                time.sleep(wait_time)
            else:
                # 最后一个点等待
                time.sleep(0.5)

        print("\nTrajectory execution completed")

    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'bus' in locals():
            bus.disconnect()

if __name__ == "__main__":
    main()
