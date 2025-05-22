from setuptools import setup

package_name = 'dynamixel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='DYNAMIXEL control package using ROS 2 and Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_controller = dynamixel_control.dynamixel_controller:main',
            'dynamixel_current_controller = dynamixel_control.dynamixel_current_controller:main',
            'keyboard_current_publisher = dynamixel_control.keyboard_current_publisher:main',
        ],
    },
)
