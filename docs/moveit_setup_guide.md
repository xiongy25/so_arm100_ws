# So_Arm100 MoveIt配置指南

本指南详细说明了使用MoveIt Setup Assistant配置So_Arm100机器人的过程。

## 前提条件

- ROS 2 Humble
- MoveIt 2
- So_Arm100 URDF模型

## 安装MoveIt

```bash
sudo apt update
sudo apt install ros-humble-moveit
```

## 启动MoveIt Setup Assistant

```bash
source /opt/ros/humble/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## 配置步骤

### 1. 创建新的配置包

1. 点击"Create New MoveIt Configuration Package"
2. 加载URDF文件：
   - 路径：`/home/ubuntu22/ros2_so_arm100/so_arm100/urdf/so_arm100.urdf`
   - 点击"Load Files"

### 2. 自碰撞检测

1. 点击左侧的"Self-Collisions"
2. 点击"Generate Collision Matrix"
3. 等待碰撞矩阵生成完成

### 3. 规划组配置

#### 机械臂组
1. 点击左侧的"Planning Groups"
2. 点击"Add Group"
3. 配置：
   - 组名称：`arm`
   - 运动学求解器：`KDLKinematicsPlugin`
   - 组类型：`Chain`
   - 基座链接：`Base`
   - 末端链接：`wrist_2_link`


#### 夹持器组
1. 点击"Add Group"
2. 配置：
   - 组名称：`gripper`
   - 运动学求解器：`KDLKinematicsPlugin`
   - 组类型：`Chain`
   - 基座链接：`wrist_2_link`
   - 末端链接：`jaw_link`


### 4. 机器人位姿

1. 点击左侧的"Robot Poses"
2. 点击"Add Pose"
3. 配置：
   - 位姿名称：`home`
   - 规划组：`arm`
   - 将所有关节角度设置为0
4. 点击"Save"保存

### 5. 末端执行器

1. 点击左侧的"End Effectors"
2. 点击"Add End Effector"
3. 配置：
   - 名称：`gripper`
   - 末端执行器组：`gripper`
   - 父级链接：`wrist_2_link`
   - 父级组：`arm`
4. 点击"Save"保存

### 6. 虚拟关节

1. 点击左侧的"Virtual Joints"
2. 点击"Add Virtual Joint"
3. 配置：
   - 关节名称：`fixed_base`
   - 子级链接：`Base`
   - 父级坐标系名称：`world`
   - 关节类型：`fixed`
4. 点击"Save"保存

### 7. 控制器配置

#### ROS 2控制器
1. 点击左侧的"ROS 2 Controllers"
2. 点击"Add Controller"
3. 配置机械臂控制器：
   - 控制器名称：`arm_controller`
   - 控制器类型：`joint_trajectory_controller/JointTrajectoryController`
   - 选择以下关节：
     - shoulder_pan_joint
     - shoulder_lift_joint
     - elbow_joint
     - wrist_pitch_joint
     - wrist_roll_joint
   - 在"Controller Settings"中：
     - Command Interface: 选择 `position`
     - State Interface: 选择 `position` 和 `velocity`
4. 点击"Save"保存

5. 点击"Add Controller"
6. 配置夹持器控制器：
   - 控制器名称：`gripper_controller`
   - 控制器类型：`joint_trajectory_controller/JointTrajectoryController`
   - 选择关节：`jaw_joint`
   - 在"Controller Settings"中：
     - Command Interface: 选择 `position`
     - State Interface: 选择 `position` 和 `velocity`
7. 点击"Save"保存

#### MoveIt控制器
1. 点击左侧的"MoveIt Controllers"
2. 在"Planning Group"下拉菜单中选择"arm"
3. 配置机械臂控制器：
   - Controller Name: `arm_controller`
   - Action NS: 留空
   - Default: 勾选
   - Joints: 确认包含所有机械臂关节
4. 点击"Add Controller"保存

5. 在"Planning Group"下拉菜单中选择"gripper"
6. 配置夹持器控制器：
   - Controller Name: `gripper_controller`
   - Action NS: 留空
   - Default: 勾选
   - Joints: 确认包含夹持器关节
7. 点击"Add Controller"保存

### 8. 生成配置文件

1. 点击左侧的"Configuration Files"
2. 填写：
   - 作者姓名：[你的姓名]
   - 作者邮箱：[你的邮箱]
   - 包名称：`so_arm100_moveit_config`
3. 点击"Generate Package"

## 构建和测试

生成配置包后：

1. 构建工作空间：
```bash
cd /home/ubuntu22/ros2_so_arm100
colcon build
```

2. 加载环境变量：
```bash
source install/setup.bash
```

3. 启动演示：
```bash
ros2 launch so_arm100_moveit_config demo.launch.py
```

## 故障排除

如果在RViz中看不到机器人：
1. 检查RViz中的Fixed Frame是否设置为"world"
2. 验证TF树是否正确发布（使用`ros2 run tf2_tools view_frames`）
3. 确认robot_description话题是否正在发布
