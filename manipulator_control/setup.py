import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'manipulator_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oleksandr',
    maintainer_email='proskurin1408@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tactical_server = manipulator_control.tactical_server:main',
            'control_hub = manipulator_control.control_hub:main',
            'debug_moveit = manipulator_control.debug_moveit:main',
            'workspace_mapper = manipulator_control.workspace_mapper:main',
            'teleop_manager = manipulator_control.teleop_manager:main',
        ],
    },
)
