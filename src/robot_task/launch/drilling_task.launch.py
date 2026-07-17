from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    matrix_publisher_node = Node(
        package='robot_task',
        executable='matrix_publisher',
        name='matrix_publisher',
        output='screen',
    )

    drill_context_builder_node = Node(
        package='robot_task',
        namespace='drilling_task',
        executable='drill_context_builder',
        name='drill_context_builder',
        output='screen',
    )

    drill_executor_node = Node(
        package='robot_task',
        namespace='drilling_task',
        executable='drill_executor',
        name='drill_executor',
        output='screen',
    )

    return LaunchDescription([
        matrix_publisher_node,
        drill_context_builder_node,
        drill_executor_node,
    ])
