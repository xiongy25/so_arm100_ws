from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Initialize MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder("so_arm100", package_name="so_arm100_moveit_config")
    
    # 设置 URDF 文件路径
    xacro_file = os.path.join(get_package_share_directory("so_arm100"), "urdf", "so_arm100_control.urdf.xacro")
    moveit_config.robot_description_file = xacro_file
    
    # 构建配置
    configs = moveit_config.to_moveit_configs()
    
    return generate_demo_launch(configs)
