from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    graph_client_node = Node(
        package='robot_task',
        namespace='painting_task',
        executable='graph_client',
        name='graph_client',
        output='screen',
    )

    return LaunchDescription([
        graph_client_node,
    ])
