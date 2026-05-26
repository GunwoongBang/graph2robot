import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_gazebo')

    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join(filter(None, [
        os.environ.get('GAZEBO_MODEL_PATH', ''),
        os.path.join(package_share, 'models', 'robots'),
        os.path.dirname(package_share),
    ]))

    empty_world = os.path.join(
        package_share, 'models', 'worlds', 'empty_world.sdf')

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
        world_spawner,
        robot_spawner,
    ])
