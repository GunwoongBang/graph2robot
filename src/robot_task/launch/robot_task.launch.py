from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    graph_node = Node(
        package='robot_task',
        executable='graph_client',
        name='graph_client',
        output='screen',
    )

    task_node = Node(
        package='robot_task',
        executable='task_publisher',
        name='task_publisher',
        output='screen',
    )

    return LaunchDescription([
        graph_node,
        task_node,
    ])
