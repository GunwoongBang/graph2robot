from setuptools import find_packages, setup

package_name = 'robot_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/drilling_task.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/transform_matrix.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunwoong',
    maintainer_email='gw.bang@tum.de',
    description='Task management for robot simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager=robot_task.task_manager:main',
            'task_generator=robot_task.task_generator:main',
            'task_evaluator=robot_task.task_evaluator:main',
            'drill_context_builder=robot_task.drill_context_builder:main',
            'drill_executor=robot_task.drill_executor:main',
        ],
    },
)
