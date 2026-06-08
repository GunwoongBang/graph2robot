from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    graph_client_node = Node(
        package='robot_task',
        namespace='drilling_task',
        executable='graph_client',
        name='graph_client',
        output='screen',
    )

    task_manager_node = Node(
        package='robot_task',
        namespace='drilling_task',
        executable='task_manager',
        name='task_manager',
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
        graph_client_node,
        task_manager_node,
        drill_context_builder_node,
        drill_executor_node,
    ])
