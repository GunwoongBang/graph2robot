from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('robot_rviz'))
    rviz_config = str(package_share / 'config' / 'config.rviz')

    # MoveIt parameters so the MotionPlanning panel can resolve the
    # PlanningGroup, robot model, and planning scene.
    moveit_config = (
        MoveItConfigsBuilder('husky_ur5e',
                             package_name='husky_ur5e_moveit_config')
        .robot_description(file_path='config/husky_ur5e.urdf.xacro')
        .robot_description_semantic(file_path='config/husky_ur5e.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    pointcloud_publisher_node = Node(
        package='robot_rviz',
        namespace='robot_rviz',
        executable='pointcloud_publisher',
        name='pointcloud_publisher',
        output='screen',
    )

    scene_graph_builder_node = Node(
        package='robot_rviz',
        namespace='scene_graph',
        executable='scene_graph_builder',
        name='scene_graph_builder',
        output='screen',
    )

    task_distributor_node = Node(
        package='robot_rviz',
        namespace='robot_rviz',
        executable='task_distributor',
        name='task_distributor',
        output='screen',
    )

    task_representer_node = Node(
        package='robot_rviz',
        namespace='robot_rviz',
        executable='task_representer',
        name='task_representer',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        pointcloud_publisher_node,
        scene_graph_builder_node,
        task_distributor_node,
        task_representer_node,
        rviz,
    ])
