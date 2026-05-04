from setuptools import find_packages, setup

package_name = 'robot_graph'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_graph.launch.py'
        ]),
    ],
    install_requires=['setuptools', 'neo4j', 'python-dotenv'],
    zip_safe=True,
    maintainer='gunwoong',
    maintainer_email='gw.bang@tum.de',
    description='Graph management for robot simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'graph_server=robot_graph.graph_server:main',
        ],
    },
)
