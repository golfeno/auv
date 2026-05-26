from setuptools import setup
import os
from glob import glob

package_name = 'my_auv_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=['my_auv_control',  'auv_nav'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='golfe',
    maintainer_email='your_email@example.com',
    description='AUV Control & OOP Autopilot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopilot = auv_nav.autopilot_node:main',
            'fake_barometer = my_auv_control.fake_barometer:main',
            'mixer = my_auv_control.auv_control_mixer:main',
        ],
    },
)
