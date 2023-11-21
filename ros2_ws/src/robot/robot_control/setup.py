import os
from setuptools import find_packages, setup

package_name = 'robot'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    author='Renan Matsuda',
    author_email='renan.matsuda@usp.br',
    description='Python packages for neuronavigation',
    license='TODO',
    entry_points={
        'console_scripts': [
            'start = robot.robot:main',
        ],
    },
)
