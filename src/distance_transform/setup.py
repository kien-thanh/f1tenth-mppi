from setuptools import find_packages, setup

package_name = 'distance_transform'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kien-Thanh Tran',
    maintainer_email='kien-thanh.tran@outlook.com',
    description='Converts the global reference path into distance and heading grids that the MPPI cost function samples on the GPU.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'distance_transform = distance_transform.distance_transform:main'
        ],
    },
)
