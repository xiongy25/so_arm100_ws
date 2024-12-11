from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    package_path = get_package_share_path('so_arm100')
    urdf_path = os.path.join(package_path, 'urdf', 'so_arm100.urdf')
    
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Add static transform publisher for map to world
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )

    # Add static transform publisher for world to Base
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'Base']
    )

    rviz_config_file = os.path.join(
        package_path,
        'config',
        'urdf.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        static_tf_map,
        static_tf_world,
        rviz_node
    ])
