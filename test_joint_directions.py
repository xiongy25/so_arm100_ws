#!/usr/bin/env python3

from motors.feetech import FeetechMotorsBus
import time

def main():
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
        # 初始化总线
        bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
        print("连接到电机总线...")
        bus.connect()

        # 等待电机准备就绪
        time.sleep(1)

        # 启用扭矩
        print("\n启用扭矩...")
        bus.write("Torque_Enable", [1] * len(motors))
        time.sleep(0.5)

        # 设置P系数（控制精度）
        print("\n设置P系数...")
        bus.write("P_Coefficient", [32] * len(motors))
        time.sleep(0.5)

        # 将所有关节移动到中心位置（零位）
        print("\n将所有关节移动到零位（中心位置）...")
        bus.write("Goal_Position", [2048] * len(motors))
        time.sleep(3)  # 等待所有关节到达零位位置

        # 读取当前位置确认
        print("\n读取当前位置...")
        positions = bus.read("Present_Position")
        print(f"当前位置: {positions}")

        while True:
            # 显示菜单
            print("\n=== 关节方向测试程序 ===")
            print("可用的关节:")
            for i, joint_name in enumerate(motors.keys(), 1):
                print(f"{i}. {joint_name}")
            print("0. 退出程序")

            # 获取用户选择
            choice = input("\n请选择要测试的关节 (0-6): ")
            if choice == '0':
                break

            try:
                joint_index = int(choice) - 1
                if joint_index < 0 or joint_index >= len(motors):
                    print("无效的选择！")
                    continue

                joint_name = list(motors.keys())[joint_index]
                print(f"\n测试 {joint_name}:")
                print("1. 正向转动 (+10度)")
                print("2. 反向转动 (-10度)")
                print("3. 返回零位")
                print("0. 返回主菜单")

                action = input("请选择动作 (0-3): ")
                if action == '1':
                    print(f"正向转动 {joint_name} +10度")
                    current_pos = bus.read("Present_Position", joint_name)
                    bus.write("Goal_Position", current_pos + 100, joint_name)  # 约10度
                elif action == '2':
                    print(f"反向转动 {joint_name} -10度")
                    current_pos = bus.read("Present_Position", joint_name)
                    bus.write("Goal_Position", current_pos - 100, joint_name)  # 约10度
                elif action == '3':
                    print(f"将 {joint_name} 返回零位")
                    bus.write("Goal_Position", 2048, joint_name)  # 回到中心位置
                elif action == '0':
                    continue
                else:
                    print("无效的选择！")
                    continue

                time.sleep(1)  # 等待运动完成

            except ValueError:
                print("请输入有效的数字！")

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if 'bus' in locals():
            # 禁用扭矩
            print("\n禁用扭矩...")
            bus.write("Torque_Enable", [0] * len(motors))
            time.sleep(0.5)
            
            print("断开与电机的连接...")
            bus.disconnect()

if __name__ == "__main__":
    main()
