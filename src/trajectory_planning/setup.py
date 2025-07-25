from setuptools import setup

package_name = 'trajectory_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='secret-base',
    maintainer_email='st176111@stud.uni-stuttgart.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_planning_node = trajectory_planning.trajectory_planning_node:main',
            'waypoint_logger_node = trajectory_planning.waypoint_logger_node:main',
        ],
    },
)
