#!/usr/bin/env python3
使用ps4手柄控制机器人

# 初始化 pygame
import pygame
import time
from motors.feetech import FeetechMotorsBus

# 初始化 pygame
pygame.init()

# 初始化手柄
# 检查是否有手柄连接
pygame.joystick.init()
if pygame.joystick.get_count() < 1:
    raise Exception("No joystick found")

# 初始化第一个手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 定义电机配置和限制
motors = {
    "shoulder_pan_joint": (1, "sts3215"),
    "shoulder_lift_joint": (2, "sts3215"),
    "elbow_joint": (3, "sts3215"),
    "wrist_pitch_joint": (4, "sts3215"),
    "wrist_roll_joint": (5, "sts3215"),
    "jaw_joint": (6, "sts3215")
}

joint_limits = {
    "shoulder_pan_joint": (-1.57, 1.57),
    "shoulder_lift_joint": (-1.57, 1.57),
    "elbow_joint": (-1.57, 1.57),
    "wrist_pitch_joint": (-1.57, 1.57),
    "wrist_roll_joint": (-1.57, 1.57),
    "jaw_joint": (-0.7, 0.7)
}

# 将弧度转换为步数的函数
def rad_to_steps(rad):
    degree = rad * 180.0 / 3.14159
    steps = int(2048 + (degree * 4096 / 360.0))
    return steps

# 将步数转换为弧度的函数
def steps_to_rad(steps):
    degree = (steps - 2048) * 360.0 / 4096
    rad = degree * 3.14159 / 180.0
    return rad

# 初始化电机总线
bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
print("Connecting to motors bus...")
bus.connect()
time.sleep(1)
print("Enabling torque...")

# 定义按键到关节的映射
key_to_joint = {
    "dpad_up": "shoulder_pan_joint",
    "dpad_down": "shoulder_pan_joint",
    "dpad_left": "shoulder_lift_joint",
    "dpad_right": "shoulder_lift_joint",
    "right_dpad_up": "elbow_joint",
    "right_dpad_down": "elbow_joint",
    "right_dpad_left": "wrist_pitch_joint",
    "right_dpad_right": "wrist_pitch_joint",
    "L1": "wrist_roll_joint",
    "L2": "wrist_roll_joint",
    "R1": "jaw_joint",
    "R2": "jaw_joint"
}

# 定义按键到动作的映射
key_to_action = {
    "dpad_up": 1,
    "dpad_down": -1,
    "dpad_left": -1,
    "dpad_right": 1,
    "right_dpad_up": 1,
    "right_dpad_down": -1,
    "right_dpad_left": -1,
    "right_dpad_right": 1,
    "L1": -1,
    "L2": 1,
    "R1": -1,
    "R2": 1
}

# 主控制循环
def control_loop():
    running = True
    while running:
        # 处理 pygame 事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                # 检查哪个按键被按下
                for key, joint in key_to_joint.items():
                    if joystick.get_button(pygame_controller_mapping[key]):
                        action = key_to_action[key]
                        # 获取当前关节位置
                        current_steps = bus.get_motor_position(joint)
                        # 计算新的步数
                        new_steps = current_steps + (action * 10)  # 10 是步数增量，可以调整
                        # 检查负载限制
                        load = bus.get_motor_load(joint)
                        if load < 200:
                            bus.set_motor_position(joint, new_steps)
                        else:
                            print(f"Load too high on {joint}, stopping movement.")
            elif event.type == pygame.JOYAXISMOTION:
                # 示例：将摇杆轴映射到机器人关节
                axis_value = joystick.get_axis(0)  # Example axis
                # 将轴值转换为关节限制内的弧度值
                radian_value = joint_limits["shoulder_pan_joint"][0] + (axis_value * (joint_limits["shoulder_pan_joint"][1] - joint_limits["shoulder_pan_joint"][0]))
                steps = rad_to_steps(radian_value)
                # 发送命令到电机
                bus.set_motor_position("shoulder_pan_joint", steps)

if __name__ == "__main__":
    try:
        control_loop()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # 退出 pygame
        pygame.quit()
