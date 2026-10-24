from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f1tenth_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kien-Thanh Tran',
    maintainer_email='kien-thanh.tran@outlook.com',
    description='Launch files and parameters for MPPI, AMCL localization, EKF, the nav2 global planner and the map server.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
