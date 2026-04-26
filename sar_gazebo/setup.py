from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sar_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz') + glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'aruco1'),
            ['models/aruco1/model.config', 'models/aruco1/model.sdf']),
        (os.path.join('share', package_name, 'models', 'aruco1', 'meshes'),
            glob('models/aruco1/meshes/*.obj') + glob('models/aruco1/meshes/*.mtl')),
        (os.path.join('share', package_name, 'models', 'aruco1', 'textures'),
            glob('models/aruco1/textures/*.png') + glob('models/aruco1/textures/*.jpg')),
        (os.path.join('share', package_name, 'models', 'office'),
            ['models/office/model.config', 'models/office/model.sdf']),
        (os.path.join('share', package_name, 'models', 'office', 'meshes'),
            glob('models/office/meshes/*.stl')),
        (os.path.join('share', package_name, 'models', 'ArucoBox'),
            ['models/ArucoBox/model.config', 'models/ArucoBox/model.sdf']),
        (os.path.join('share', package_name, 'models', 'ArucoBox', 'meshes'),
            glob('models/ArucoBox/meshes/*.obj')
            + glob('models/ArucoBox/meshes/*.mtl')
            + glob('models/ArucoBox/meshes/*.dae')),
        (os.path.join('share', package_name, 'models', 'ArucoBox', 'textures'),
            glob('models/ArucoBox/textures/*.png')
            + glob('models/ArucoBox/textures/*.jpg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alvaro J. Gaona',
    maintainer_email='alvgaona@gmail.com',
    description='SAR environment with ROSbot XL Autonomy in Gazebo',
    license='MIT',
    tests_require=['pytest'],
)
