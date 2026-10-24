from setuptools import setup

package_name = 'safety_node'

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
    description='Emergency braking based on the instantaneous time-to-collision from LiDAR scans.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = safety_node.safety_node:main'
        ],
    },
)
