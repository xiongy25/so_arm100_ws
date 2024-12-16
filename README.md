# SO-ARM100 真实机械臂控制指南

## 1. 简介

本指南旨在帮助用户正确配置和控制SO-ARM100机械臂。指南包含两种控制方式：
1. 通过ROS2直接控制
2. 通过MoveIt规划控制

## 2. 系统要求

### 2.1 硬件要求
- SO-ARM100机械臂
- USB转TTL模块
- 电源适配器

### 2.2 软件要求
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

## 3. 基础配置

### 3.1 USB设备配置
1. 连接USB设备
2. 设置权限：
```bash
sudo chmod 666 /dev/ttyACM0
```

### 3.2 ROS2工作空间配置
1. 确保ROS2环境已经source
2. 编译工作空间：
```bash
cd ~/so_arm100_ws
colcon build
```

## 4. 直接控制模式

### 4.1 使用Python脚本直接控制
1. 运行测试脚本：
```bash
python3 test_arm_direct_2.py
```

2. 关节配置说明：
- shoulder_pan_joint: ID 1
- shoulder_lift_joint: ID 2
- elbow_joint: ID 3
- wrist_pitch_joint: ID 4
- wrist_roll_joint: ID 5
-jaw_joint: ID 6

## 5. MoveIt控制模式

### 5.1 使用MoveIt直接控制（需要硬件接口正常工作）
1. 启动MoveIt：
```bash
source install/setup.bash
ros2 launch so_arm100_moveit_config demo.launch.py
```

2. 使用RViz界面进行控制

### 5.2 使用MoveIt轨迹离线控制

当硬件接口未完全就绪时，可以采用轨迹记录和回放的方式控制机械臂。

#### 5.2.1 工作原理
1. 使用MoveIt生成运动规划轨迹
2. 记录轨迹数据
3. 使用直接控制方式执行记录的轨迹

#### 5.2.2 使用步骤

1. 启动轨迹记录器（需退出conda环境）：
```bash
python3 record_moveit_trajectory.py
```

2. 使用MoveIt规划轨迹（轨迹会自动保存到`recorded_trajectories`目录）

3. 执行记录的轨迹：
```bash
python3 execute_recorded_trajectory.py recorded_trajectories/trajectory_20241216_203810_part1.json
```

#### 5.2.3 轨迹文件格式
```json
{
  "joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                  "wrist_pitch_joint", "wrist_roll_joint", "jaw_joint"],
  "points": [
    {
      "positions": [0.0, -1.57, 1.57, 0.0, 0.0, 0.0],
      "time_from_start": 1.0
    }
  ]
}
```

### 5.3 测试和校准机器人关节方向

在使用MoveIt控制真实机器人之前，需要确保URDF文件中定义的关节转动方向与真实机器人一致。这一步很重要，因为如果方向不一致，机器人的实际运动将与规划的轨迹相反。

#### 5.3.1 了解真实机器人的运动特性

SO-ARM100机器人的所有关节都遵循以下规则：
- 给电机正值：关节顺时针转动
- 给电机负值：关节逆时针转动
- 电机中心位置（零位）：2048

#### 5.3.2 测试URDF中的关节方向

1. 启动RViz中的机器人模型：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch so_arm100 display.launch.py
```

2. 在RViz界面中：
   - 使用joint_state_publisher_gui控制每个关节
   - 记录每个关节在给正值时的转动方向
   - 与真实机器人的特性对比（正值应该是顺时针转动）

3. 修改URDF文件：
   - 如果某个关节在RViz中给正值时是逆时针转动，需要修改该关节的轴向
   - 修改方法：在URDF文件中找到对应关节的`<axis>`标签
   - 将`xyz`属性的值取反，例如：
     - 原始值：`<axis xyz="0 0 1"/>`
     - 修改为：`<axis xyz="0 0 -1"/>`

#### 5.3.3 使用测试程序验证真实机器人方向

我们提供了一个专门的测试程序来验证真实机器人的关节方向：

```bash
python3 test_joint_directions.py
```

这个程序可以：
1. 自动将机器人移动到零位（中心位置2048）
2. 允许你选择要测试的关节
3. 可以控制关节正向转动（+10度）或反向转动（-10度）
4. 随时可以让关节返回零位

使用这个程序确认：
1. 所有关节在给正值时都是顺时针转动
2. 所有关节都能正确回到零位（中心位置）
3. 关节的实际运动方向与URDF中的定义一致

#### 5.3.4 方向校准检查清单

对每个关节进行以下检查：
- [ ] shoulder_pan_joint（关节1）：正值顺时针转动
- [ ] shoulder_lift_joint（关节2）：正值顺时针转动
- [ ] elbow_joint（关节3）：正值顺时针转动
- [ ] wrist_pitch_joint（关节4）：正值顺时针转动
- [ ] wrist_roll_joint（关节5）：正值顺时针转动
- [ ] jaw_joint（关节6）：正值顺时针转动

只有在确保所有关节方向都正确之后，才能开始使用MoveIt进行轨迹规划和执行。

## 6. 故障排除

### 6.1 常见问题
1. USB设备无法识别
   - 检查USB连接
   - 确认设备权限

2. 机械臂不响应
   - 检查电源连接
   - 验证串口配置

3. MoveIt规划失败
   - 检查规划参数
   - 确认目标位置是否可达

### 6.2 调试工具
1. 使用`test_arm_direct_2.py`测试基本控制
2. 检查`recorded_trajectories`目录下的轨迹文件
3. 使用RViz可视化机械臂状态

## 7. 最佳实践

### 7.1 安全建议
1. 始终在机械臂周围保持安全距离
2. 首次执行新轨迹时降低速度
3. 确保工作空间内无障碍物

### 7.2 维护建议
1. 定期检查机械臂各关节
2. 保持固件更新
3. 备份重要的轨迹文件
4. 记录异常情况以便追踪

## 8. 相关文件说明

### 8.1 控制脚本
- `test_arm_direct_2.py`: 基础控制测试脚本
- `record_moveit_trajectory.py`: 轨迹记录脚本
- `execute_recorded_trajectory.py`: 轨迹执行脚本

### 8.2 配置文件
- `config/`: MoveIt配置文件目录
- `urdf/`: 机械臂模型文件目录

## 9. 技术支持

如遇到问题，请：
1. 查阅本指南的故障排除部分
2. 检查日志文件
3. 联系技术支持团队

---
最后更新：2024-12-11
