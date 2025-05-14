from setuptools import setup, find_packages

setup(
    name='common_utils',
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/common_utils']),
        ('share/common_utils', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='secret-base',
    maintainer_email='st176111@stud.uni-stuttgart.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

