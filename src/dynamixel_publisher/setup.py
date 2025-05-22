from setuptools import find_packages, setup

package_name = 'dynamixel_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yt6',
    maintainer_email='kikuchi@irlab-uu.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_node = dynamixel_publisher.dynamixel_node:main',
            'dynamixel_curremt_node = dynamixel_publisher.dynamixel_current_node:main',
            'keyboard_current_publisher = dynamixel_publisher.keyboard_current_publisher:main',
        ],
    },
)
