from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_auv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='AUV control with dual turn',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv_telemetry = my_auv_control.auv_telemetry:main',
            'auv_control_mixer = my_auv_control.auv_control_mixer:main',
            'keyboard_teleop = my_auv_control.keyboard_teleop:main',
            'dual_turn_control = my_auv_control.dual_turn_control:main',
            'pitch_stabilizer = my_auv_control.pitch_stabilizer:main',
        ],
    },
)
