import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vehicle_model'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu22',
    maintainer_email='2578034340@qq.com',
    description='Bicycle vehicle model for WUTA-FSD closed-loop simulation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vehicle_model = vehicle_model.vehicle_model:main',
        ],
    },
)
