import os
from glob import glob
from setuptools import setup

package_name = 'storm32_gimbal'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ethan Tran',
    maintainer_email='entran@ncsu.edu',
    description='Driver for the STorM32 gimbal controller.',
    license='GPLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gimbal = storm32_gimbal.storm32_node:main'
        ],
    },
)
