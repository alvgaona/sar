from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sar_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alvaro J. Gaona',
    maintainer_email='alvgaona@gmail.com',
    description='Mission control for the SAR robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore_and_detect = sar_mission.explore_and_detect:main',
        ],
    },
)
