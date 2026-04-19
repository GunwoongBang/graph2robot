from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_simulation')
    rviz_config = package_share + '/rviz/pointcloud.rviz'

    pointcloud_node = Node(
        package='robot_simulation',
        executable='pointcloud_publisher',
        name='pointcloud_publisher',
        output='screen',
    )

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
    )
    return LaunchDescription([
        pointcloud_node,
        rviz,
    ])
