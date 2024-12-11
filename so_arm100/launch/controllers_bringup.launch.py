from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os

startup_controllers = [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "gripper_controller",
]

def generate_launch_description():
    pkg_path = get_package_share_path("so_arm100")
    
    # Declare the hardware_type argument
    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="mock_components",
        description="Hardware type to use (mock_components or real)"
    )
    
    # Load URDF with hardware type parameter
    xacro_file = os.path.join(pkg_path, "urdf", "so_arm100.urdf.xacro")
    robot_description = Command(
        ['xacro ', xacro_file, ' hardware_type:=', LaunchConfiguration('hardware_type')]
    )
    
    # Load controllers
    ros2_controllers_file = os.path.join(pkg_path, "control", "ros2_controllers.yaml")

    return LaunchDescription([
        # Include the hardware_type argument
        hardware_type_arg,
        
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        
        # Controller Manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[
                {"robot_description": robot_description},
                ros2_controllers_file,
            ],
            output="screen",
        ),
        
        # Load Controllers
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_joint_state_broadcaster",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_joint_trajectory_controller",
            arguments=["joint_trajectory_controller"],
            output="screen",
        ),
        
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_gripper_controller",
            arguments=["gripper_controller"],
            output="screen",
        ),
    ])
