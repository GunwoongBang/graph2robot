from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_task',
            executable='task_publisher',
            name='task_publisher',
            output='screen',
        ),
        Node(
            package='robot_task',
            executable='matrix_publisher',
            name='matrix_publisher',
            output='screen',
        ),
    ])
