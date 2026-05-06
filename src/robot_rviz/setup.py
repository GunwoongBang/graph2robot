from setuptools import find_packages, setup

package_name = 'robot_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_rviz.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/config.rviz',
        ]),
        ('share/' + package_name + '/models', [
            'models/cloudGlobal_cleaned_excluded.pcd',
            'models/cloudGlobal_cleaned_excluded.csv',
        ]),
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
            'pointcloud_publisher=robot_rviz.pointcloud_publisher:main',
            'task_subscriber=robot_rviz.task_subscriber:main',
        ],
    },
)
