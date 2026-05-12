from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('robot_rviz'))
    rviz_config = package_share / 'config' / 'config.rviz'

    pointcloud_publisher_node = Node(
        package='robot_rviz',
        executable='pointcloud_publisher',
        name='pointcloud_publisher',
        output='screen',
    )

    task_distributor_node = Node(
        package='robot_rviz',
        executable='task_distributor',
        name='task_distributor',
        output='screen',
    )

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        pointcloud_publisher_node,
        task_distributor_node,
        rviz,
    ])
