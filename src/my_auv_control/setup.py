from setuptools import setup

package_name = 'my_auv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf2_ros', 'geometry_msgs', 'nav_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='golfe',
    maintainer_email='golfe@example.com',
    description='AUV Control Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv_pid_nav = my_auv_control.auv_pid_nav:main',
            'auv_test_straight = my_auv_control.auv_test_straight:main',
            'fake_barometer = my_auv_control.fake_barometer:main',
        ],
    },
)
