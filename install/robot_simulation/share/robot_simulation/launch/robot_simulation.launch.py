import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_simulation')
    world_sdf = package_share + '/worlds/world.sdf'
    rviz_config = package_share + '/rviz/pointcloud.rviz'

    gazebo_env = os.environ.copy()
    gazebo_env['GAZEBO_MODEL_PATH'] = os.path.join(package_share, 'models')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_sdf],
        env=gazebo_env,
        output='screen',
    )

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
        gazebo,
        pointcloud_node,
        rviz,
    ])
