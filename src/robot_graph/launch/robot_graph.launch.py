from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    graph_server_node = Node(
        package='robot_graph',
        executable='graph_server',
        name='graph_server',
        output='screen',
    )

    return LaunchDescription([
        graph_server_node,
    ])
