import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_gazebo')
    world_sdf = package_share + '/models/world/world.sdf'

    gazebo_env = os.environ.copy()
    gazebo_env['GAZEBO_MODEL_PATH'] = os.path.join(package_share, 'models')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_sdf],
        env=gazebo_env,
        output='screen',
    )

    return LaunchDescription([
        gazebo,
    ])
