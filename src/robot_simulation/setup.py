from setuptools import find_packages, setup

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/robot_simulation.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/pointcloud.rviz']),
        ('share/' + package_name + '/worlds/meshes',
         ['worlds/meshes/mesh.obj']),
        ('share/' + package_name + '/worlds',
         ['worlds/cloudGlobal_cleaned_excluded.pcd', 'worlds/world.sdf']),
        ('share/' + package_name + '/models/robot',
         ['models/robot/model.config', 'models/robot/robot.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunwoong',
    maintainer_email='gw.bang@tum.de',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pointcloud_publisher = robot_simulation.pointcloud_publisher:main',
        ],
    },
)
