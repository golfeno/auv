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
            'dual_turn_control = my_auv_control.dual_turn_control:main',
            'pitch_stabilizer = my_auv_control.pitch_stabilizer:main',
        ],
    },
)
