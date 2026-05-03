from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_task',
            executable='graph_client',
            name='graph_client',
            output='screen',
        ),
        Node(
            package='robot_task',
            executable='task_manager',
            name='task_manager',
            output='screen',
        ),
    ])
