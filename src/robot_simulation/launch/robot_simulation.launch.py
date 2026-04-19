import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_simulation')
    gazebo_launch = os.path.join(
        package_share, 'launch', 'robot_gazebo.launch.py')
    rviz_launch = os.path.join(
        package_share, 'launch', 'robot_rviz.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch))

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch))

    return LaunchDescription([
        gazebo,
        rviz,
    ])
