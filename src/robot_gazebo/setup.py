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
            'models/worlds/empty_world.sdf',
        ]),
        # ('share/' + package_name + '/urdf/husky_ur5e', [
        #     'urdf/husky_ur5e/husky_ur5e.urdf.xacro',
        # ]),
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
        ('share/' + package_name + '/models/robots', [
            'models/robots/husky_ur5e.urdf',
        ]),
        ('share/' + package_name + '/models/robots/a200_meshes/collision', [
            'models/robots/a200_meshes/collision/base_link.stl',
            'models/robots/a200_meshes/collision/top_chassis.stl',
            'models/robots/a200_meshes/collision/wheel.stl',
        ]),
        ('share/' + package_name + '/models/robots/a200_meshes/visual', [
            'models/robots/a200_meshes/visual/base_link.dae',
            'models/robots/a200_meshes/visual/top_chassis.dae',
            'models/robots/a200_meshes/visual/wheel.dae',
        ]),
        ('share/' + package_name + '/models/robots/ur5e_meshes/collision', [
            'models/robots/ur5e_meshes/collision/base.stl',
            'models/robots/ur5e_meshes/collision/forearm.stl',
            'models/robots/ur5e_meshes/collision/shoulder.stl',
            'models/robots/ur5e_meshes/collision/upperarm.stl',
            'models/robots/ur5e_meshes/collision/wrist1.stl',
            'models/robots/ur5e_meshes/collision/wrist2.stl',
            'models/robots/ur5e_meshes/collision/wrist3.stl',
        ]),
        ('share/' + package_name + '/models/robots/ur5e_meshes/visual', [
            'models/robots/ur5e_meshes/visual/base.dae',
            'models/robots/ur5e_meshes/visual/forearm.dae',
            'models/robots/ur5e_meshes/visual/shoulder.dae',
            'models/robots/ur5e_meshes/visual/upperarm.dae',
            'models/robots/ur5e_meshes/visual/wrist1.dae',
            'models/robots/ur5e_meshes/visual/wrist2.dae',
            'models/robots/ur5e_meshes/visual/wrist3.dae',
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
            'robot_spawner=robot_gazebo.robot_spawner:main',
            'world_spawner=robot_gazebo.world_spawner:main',
        ],
    },
)
