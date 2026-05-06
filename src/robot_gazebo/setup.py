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
        ('share/' + package_name + '/models/robots/husky', [
            'models/robots/husky/husky.urdf',
            'models/robots/husky/model.config',
        ]),
        ('share/' + package_name + '/models/robots/husky/meshes', [
            'models/robots/husky/meshes/base_link.stl',
            'models/robots/husky/meshes/bumper.stl',
            'models/robots/husky/meshes/top_plate.stl',
            'models/robots/husky/meshes/user_rail.stl',
            'models/robots/husky/meshes/wheel.stl',
        ]),
        ('share/' + package_name + '/models/robots/ur5e', [
            'models/robots/ur5e/ur5e.urdf',
            'models/robots/ur5e/model.config',
        ]),
        ('share/' + package_name + '/models/robots/ur5e/meshes/collision', [
            'models/robots/ur5e/meshes/collision/base.stl',
            'models/robots/ur5e/meshes/collision/forearm.stl',
            'models/robots/ur5e/meshes/collision/shoulder.stl',
            'models/robots/ur5e/meshes/collision/upperarm.stl',
            'models/robots/ur5e/meshes/collision/wrist1.stl',
            'models/robots/ur5e/meshes/collision/wrist2.stl',
            'models/robots/ur5e/meshes/collision/wrist3.stl',
        ]),
        ('share/' + package_name + '/models/robots/ur5e/meshes/visual', [
            'models/robots/ur5e/meshes/visual/base.dae',
            'models/robots/ur5e/meshes/visual/forearm.dae',
            'models/robots/ur5e/meshes/visual/shoulder.dae',
            'models/robots/ur5e/meshes/visual/upperarm.dae',
            'models/robots/ur5e/meshes/visual/wrist1.dae',
            'models/robots/ur5e/meshes/visual/wrist2.dae',
            'models/robots/ur5e/meshes/visual/wrist3.dae',
        ]),
        # === The 'yn' robot is for testing only, now deprecated ===
        # ('share/' + package_name + '/models/robots/yn', [
        #     'models/robots/yn/robot.sdf',
        #     'models/robots/yn/model.config',
        # ]),
        # ('share/' + package_name + '/models/robots/yn/meshes/DAE/', [
        #     'models/robots/yn/meshes/DAE/base_link.dae',
        #     'models/robots/yn/meshes/DAE/camera_stay.dae',
        #     'models/robots/yn/meshes/DAE/camera.dae',
        #     'models/robots/yn/meshes/DAE/front_left_sus.dae',
        #     'models/robots/yn/meshes/DAE/front_right_sus.dae',
        #     'models/robots/yn/meshes/DAE/front_wheel.dae',
        #     'models/robots/yn/meshes/DAE/rear_wheel.dae',
        # ]),
        # ('share/' + package_name + '/models/robots/yn/meshes/STL/', [
        #     'models/robots/yn/meshes/STL/base_link.stl',
        #     'models/robots/yn/meshes/STL/camera.stl',
        #     'models/robots/yn/meshes/STL/front_left_sus.stl',
        #     'models/robots/yn/meshes/STL/front_right_sus.stl',
        #     'models/robots/yn/meshes/STL/front_wheel.stl',
        #     'models/robots/yn/meshes/STL/rear_wheel.stl',
        # ]),
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
