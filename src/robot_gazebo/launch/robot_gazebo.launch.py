import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_gazebo')

    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join(filter(None, [
        os.environ.get('GAZEBO_MODEL_PATH', ''),
        os.path.join(package_share, 'models', 'robots'),
        os.path.dirname(package_share),
    ]))

    empty_world = os.path.join(
        package_share, 'models', 'worlds', 'empty_world.sdf')

    # Build the unified MoveIt config (URDF + SRDF + controllers + kinematics).
    moveit_config = (
        MoveItConfigsBuilder('husky_ur5e',
                             package_name='husky_ur5e_moveit_config')
        .robot_description(file_path='config/husky_ur5e.urdf.xacro')
        .robot_description_semantic(file_path='config/husky_ur5e.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )

    # gazebo_ros's launch wrapper loads gazebo_ros_init + gazebo_ros_factory as
    # system plugins, providing /spawn_entity, /delete_entity, /set_entity_state.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch',
            'gazebo.launch.py',
        )),
        launch_arguments={
            'world': empty_world,
            'verbose': 'true',
        }.items(),
    )

    robot_description = {
        'robot_description': ParameterValue(
            moveit_config.robot_description['robot_description'],
            value_type=None,
        ),
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # MoveIt's planner / scene server.
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    ur5e_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur5e_arm_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    world_spawner = Node(
        package='robot_gazebo',
        namespace='robot_gazebo',
        executable='world_spawner',
        name='world_spawner',
        output='screen',
    )

    robot_spawner = Node(
        package='robot_gazebo',
        namespace='robot_gazebo',
        executable='robot_spawner',
        name='robot_spawner',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        move_group,
        world_spawner,
        robot_spawner,
        joint_state_broadcaster_spawner,
        ur5e_arm_controller_spawner,
    ])
