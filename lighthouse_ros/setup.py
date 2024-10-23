import os
import glob

from setuptools import find_packages, setup

package_name = 'lighthouse_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/lighthouse_ros.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hitalio',
    maintainer_email='juancarosella96@creativa77.com.ar',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lighthouse_ros = lighthouse_ros.node:main'
        ],
    },
)
