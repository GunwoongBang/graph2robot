from setuptools import find_packages, setup

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_gazebo.launch.py',
        ]),
        ('share/' + package_name + '/models/worlds', [
            'models/worlds/ifc_world.sdf',
        ]),
        ('share/' + package_name + '/models/worlds/meshes', [
            'models/worlds/meshes/IfcSlab_20rBv4HojAY8M3avWLHAN_.obj',
            'models/worlds/meshes/IfcWall_1iDZV_Brn0Z9GW0EFhIoO4.obj',
            'models/worlds/meshes/IfcWall_3_Sc93izfEuuDAiaR0_6h3.obj',
            'models/worlds/meshes/IfcWall_3S3VgMDFLDiPHeIJPk5HIn.obj',
            'models/worlds/meshes/IfcWall_3S3VgMDFLDiPHeIJPk5HSc.obj',
            'models/worlds/meshes/IfcWall_3S3VgMDFLDiPHeIJPk5HTj.obj',
            'models/worlds/meshes/IfcWall_20rBv4HojAY8M3avWLHAad.obj',
            'models/worlds/meshes/IfcWall_20rBv4HojAY8M3avWLHAgu.obj',
        ]),
        ('share/' + package_name + '/models/robots/yn', [
            'models/robots/yn/model.config',
            'models/robots/yn/robot.sdf',
        ]),
        ('share/' + package_name + '/models/robots/yn/meshes/DAE/', [
            'models/robots/yn/meshes/DAE/base_link.dae',
            'models/robots/yn/meshes/DAE/camera_stay.dae',
            'models/robots/yn/meshes/DAE/camera.dae',
            'models/robots/yn/meshes/DAE/front_left_sus.dae',
            'models/robots/yn/meshes/DAE/front_right_sus.dae',
            'models/robots/yn/meshes/DAE/front_wheel.dae',
            'models/robots/yn/meshes/DAE/rear_wheel.dae',
        ]),
        ('share/' + package_name + '/models/robots/yn/meshes/STL/', [
            'models/robots/yn/meshes/STL/base_link.stl',
            'models/robots/yn/meshes/STL/camera.stl',
            'models/robots/yn/meshes/STL/front_left_sus.stl',
            'models/robots/yn/meshes/STL/front_right_sus.stl',
            'models/robots/yn/meshes/STL/front_wheel.stl',
            'models/robots/yn/meshes/STL/rear_wheel.stl',
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
        ],
    },
)
