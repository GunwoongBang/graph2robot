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

    task_generator_node = Node(
        package='robot_task',
        namespace='drilling_task',
        executable='task_generator',
        name='task_generator',
        output='screen',
    )

    task_evaluator_node = Node(
        package='robot_task',
        namespace='drilling_task',
        executable='task_evaluator',
        name='task_evaluator',
        output='screen',
    )

    return LaunchDescription([
        graph_client_node,
        task_manager_node,
        task_generator_node,
        task_evaluator_node,
    ])
