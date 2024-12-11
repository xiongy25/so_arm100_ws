# SO-ARM100 机械臂 MoveIt 控制指南

本指南将帮助你在 Ubuntu 22.04 和 ROS2 Humble 环境下配置和控制 SO-ARM100 机械臂。

## 1. 环境要求

- Ubuntu 22.04
- ROS2 Humble
- 真实的 SO-ARM100 机械臂（连接到 /dev/LeRobotFollower）

## 2. 安装依赖

首先安装必要的 ROS2 包：

```bash
sudo apt update
sudo apt install -y ros-humble-moveit ros-humble-moveit-ros-planning-interface
sudo apt install ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-position-controllers ros-humble-gripper-controllers
sudo apt install ros-humble-joint-state-publisher-gui
```

## 3. 配置 USB 权限

为确保可以访问机械臂的 USB 端口，需要配置权限：

```bash
# 添加当前用户到 dialout 组
sudo usermod -a -G dialout $USER

# 给予 USB 端口权限
sudo chmod 666 /dev/LeRobotFollower
```

注意：添加用户组后需要重新登录才能生效。

## 4. 编译工作空间

```bash
# 进入工作空间
cd ~/so_arm100_ws

# 编译
colcon build

# 加载环境
source install/setup.bash
```

## 5. 启动控制系统

需要开启三个终端，按顺序执行以下命令：

### 终端 1：启动机械臂控制器
```bash
cd ~/so_arm100_ws
source install/setup.bash
ros2 launch so_arm100_description controllers_bringup.launch.py hardware_type:=real
```

### 终端 2：启动 MoveIt 的 move_group
```bash
cd ~/so_arm100_ws
source install/setup.bash
ros2 launch so_arm100_moveit_config move_group.launch.py
```

### 终端 3：启动 RViz 可视化界面
```bash
cd ~/so_arm100_ws
source install/setup.bash
ros2 launch so_arm100_moveit_config moveit_rviz.launch.py
```

## 6. 使用 MoveIt 控制机械臂

在 RViz 界面中：

1. 在右侧面板找到 "MotionPlanning" 插件
2. 在 "Planning" 标签页中：
   - 使用交互式标记（Interactive Markers）移动机械臂末端执行器到目标位置
   - 点击 "Plan" 按钮规划路径
   - 检查规划路径是否安全
   - 点击 "Execute" 执行规划好的运动

## 7. 可选：使用关节轨迹控制器

如果需要直接控制机械臂的各个关节，可以使用关节轨迹控制器：

```bash
source install/setup.bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

## 注意事项

1. 在执行任何实际运动之前，确保：
   - 机械臂周围有足够的空间
   - 规划的路径不会造成碰撞
   - 随时准备使用紧急停止按钮

2. 如果遇到权限问题：
   - 检查 USB 端口权限
   - 确认用户已添加到 dialout 组
   - 重新登录使组权限生效

3. 常见问题解决：
   - 如果机械臂无响应，检查 USB 连接
   - 如果规划失败，尝试调整目标位置
   - 如果出现通信错误，重新启动所有终端

## 其他资源

- SO-ARM100 GitHub 仓库：https://github.com/TheRobotStudio/SO-ARM100
- MoveIt 官方文档：https://moveit.ros.org/
- ROS2 Humble 文档：https://docs.ros.org/en/humble/
