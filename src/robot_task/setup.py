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
            'launch/robot_task.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/transform_matrix.yaml',
        ]),
    ],
    install_requires=['setuptools', 'neo4j'],
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
            'task_publisher=robot_task.task_publisher:main',
            'matrix_publisher=robot_task.matrix_publisher:main',
        ],
    },
)
