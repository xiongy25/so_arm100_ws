#!/usr/bin/env python3
# 使用ps4手柄控制机器人

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
try:
    # 启用扭矩
    bus.write("Torque_Enable", [1] * len(motors))
    time.sleep(0.5)
    
    # 设置P系数（降低以使运动更平滑）
    print("Setting P coefficient...")
    bus.write("P_Coefficient", [16] * len(motors))  # 降低P系数从32到16
    time.sleep(0.5)
    
    # 设置移动速度
    print("Setting goal speed...")
    bus.write("Goal_Speed", [100] * len(motors))  # 设置适中的移动速度
    time.sleep(0.5)
    
    # 移动到中心位置（零位）
    print("Moving to center position...")
    bus.write("Goal_Position", [2048] * len(motors))
    time.sleep(3)  # 等待所有关节到达零位位置
except Exception as e:
    print(f"Error during initialization: {e}")
    raise

# 定义按键到关节的映射
key_to_joint = {
    "dpad_up": "shoulder_pan_joint",      # 十字键上：肩部平移
    "dpad_down": "shoulder_pan_joint",    # 十字键下：肩部平移
    "dpad_left": "shoulder_lift_joint",   # 十字键左：肩部抬升
    "dpad_right": "shoulder_lift_joint",  # 十字键右：肩部抬升
    "right_dpad_up": "wrist_pitch_joint", # 三角形：手腕俯仰
    "right_dpad_down": "elbow_joint",     # 叉：肘部
    "right_dpad_left": "elbow_joint",     # 方块：肘部
    "right_dpad_right": "wrist_pitch_joint", # 圆圈：手腕俯仰
    "L1": "wrist_roll_joint",             # L1：手腕旋转
    "L2": "wrist_roll_joint",             # L2：手腕旋转
    "R1": "jaw_joint",                    # R1：夹爪
    "R2": "jaw_joint"                     # R2：夹爪
}

# PS4控制器按键映射（只包含实际按键，不包含D-pad）
pygame_controller_mapping = {
    # 动作键
    "right_dpad_up": 3,    # 三角形
    "right_dpad_down": 0,  # 叉
    "right_dpad_left": 2,  # 方块
    "right_dpad_right": 1, # 圆圈
    
    # 肩部按键
    "L1": 4,          # L1
    "L2": 6,          # L2
    "R1": 5,          # R1
    "R2": 7           # R2
}

# 定义按键到动作的映射
key_to_action = {
    # D-pad方向
    "dpad_up": 1,
    "dpad_down": -1,
    "dpad_left": -1,
    "dpad_right": 1,
    
    # 动作键方向
    "right_dpad_up": -1,    # 三角形：手腕俯仰反向
    "right_dpad_down": -1,  # 叉：肘部反向
    "right_dpad_left": 1,   # 方块：肘部正向
    "right_dpad_right": 1,  # 圆圈：手腕俯仰正向
    
    # 肩部按键方向
    "L1": -1,
    "L2": 1,
    "R1": -1,
    "R2": 1
}

# 主控制循环
def control_loop():
    running = True
    step_increment = 80  # 增加步进值到80，使移动更明显
    
    # 设置所有关节的运行速度和扭矩限制
    joints = ["shoulder_lift_joint", "shoulder_pan_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint", "jaw_joint"]
    for joint in joints:
        try:
            # 读取并打印当前参数
            torque_limit = bus.read("Torque_Limit", joint)[0]
            torque_enable = bus.read("Torque_Enable", joint)[0]
            print(f"\n{joint} parameters:")
            print(f"Torque Limit: {torque_limit}")
            print(f"Torque Enable: {torque_enable}")
            
            # 设置参数
            bus.write("Torque_Enable", [1], joint)  # 确保扭矩开启
            bus.write("Torque_Limit", [1023], joint)  # 设置最大扭矩限制
            bus.write("Goal_Speed", [200], joint)  # 增加速度
        except Exception as e:
            print(f"Error setting parameters for {joint}: {e}")
    
    print("\nPS4控制器信息：")
    print(f"按键数量: {joystick.get_numbuttons()}")
    print(f"轴数量: {joystick.get_numaxes()}")
    print(f"帽子开关数量: {joystick.get_numhats()}")
    
    print("\n按键映射：")
    for key, joint in key_to_joint.items():
        if key not in ["dpad_up", "dpad_down", "dpad_left", "dpad_right"]:
            action = key_to_action[key]
            button_id = pygame_controller_mapping[key]
            print(f"{key}: 控制 {joint} (按键ID: {button_id}, 方向: {action})")

    while running:
        # 处理 pygame 事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # 获取当前D-pad状态
        hat = joystick.get_hat(0)
        x, y = hat  # x为-1(左)、0、1(右)，y为-1(下)、0、1(上)
        
        # 调试输出D-pad状态
        if x != 0 or y != 0:
            print(f"D-pad state - x: {x}, y: {y}")
        
        # 处理左右移动（控制shoulder_lift_joint）
        if x != 0:
            try:
                # 获取当前位置
                current_steps = bus.read("Present_Position", "shoulder_lift_joint")[0]
                print(f"Current position: {current_steps}")
                
                # 计算新位置
                new_steps = current_steps + (x * step_increment)
                new_steps = max(0, min(4095, new_steps))
                print(f"Moving shoulder_lift_joint from {current_steps} to {new_steps} steps (x={x})")
                
                # 重新设置参数并发送移动命令
                bus.write("Torque_Enable", [1], "shoulder_lift_joint")
                bus.write("Torque_Limit", [1023], "shoulder_lift_joint")
                bus.write("Goal_Speed", [200], "shoulder_lift_joint")
                bus.write("Goal_Position", [new_steps], "shoulder_lift_joint")
                
                # 等待稍长时间
                time.sleep(0.15)  # 给更多响应时间
                
                # 验证移动后的位置
                after_steps = bus.read("Present_Position", "shoulder_lift_joint")[0]
                print(f"Position after move: {after_steps}")
                
                # 读取负载
                load = bus.read("Present_Load", "shoulder_lift_joint")[0]
                print(f"Present Load: {load}")
                
            except Exception as e:
                print(f"Error controlling shoulder_lift_joint: {e}")
        
        # 处理上下移动（控制shoulder_pan_joint）
        if y != 0:
            try:
                current_steps = bus.read("Present_Position", "shoulder_pan_joint")[0]
                new_steps = current_steps + (y * step_increment)
                new_steps = max(0, min(4095, new_steps))
                bus.write("Torque_Enable", [1], "shoulder_pan_joint")
                bus.write("Torque_Limit", [1023], "shoulder_pan_joint")
                bus.write("Goal_Speed", [200], "shoulder_pan_joint")
                bus.write("Goal_Position", [new_steps], "shoulder_pan_joint")
                time.sleep(0.15)  # 给更多响应时间
            except Exception as e:
                print(f"Error controlling shoulder_pan_joint: {e}")
        
        # 检查所有按键的当前状态
        for key, joint in key_to_joint.items():
            if key not in ["dpad_up", "dpad_down", "dpad_left", "dpad_right"]:  # 跳过D-pad相关的映射
                try:
                    button_id = pygame_controller_mapping[key]
                    if button_id < joystick.get_numbuttons() and joystick.get_button(button_id):
                        action = key_to_action[key]
                        try:
                            # 获取当前位置
                            current_steps = bus.read("Present_Position", joint)[0]
                            print(f"Button {key} pressed - {joint} current position: {current_steps}")
                            
                            # 计算新位置
                            new_steps = current_steps + (action * step_increment)
                            new_steps = max(0, min(4095, new_steps))
                            print(f"Moving {joint} from {current_steps} to {new_steps} steps (action={action})")
                            
                            # 重新设置参数并发送移动命令
                            bus.write("Torque_Enable", [1], joint)
                            bus.write("Torque_Limit", [1023], joint)
                            bus.write("Goal_Speed", [200], joint)
                            bus.write("Goal_Position", [new_steps], joint)
                            
                            # 等待稍长时间
                            time.sleep(0.15)  # 给更多响应时间
                            
                            # 验证移动后的位置
                            after_steps = bus.read("Present_Position", joint)[0]
                            print(f"Position after move: {after_steps}")
                            
                            # 读取负载
                            load = bus.read("Present_Load", joint)[0]
                            print(f"Present Load: {load}")
                            
                        except Exception as e:
                            print(f"Error controlling {joint}: {e}")
                except Exception as e:
                    print(f"Error with button mapping for {key}: {e}")
        
        # 短暂延时以防止CPU使用率过高
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        control_loop()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # 退出 pygame
        pygame.quit()
