from setuptools import find_packages, setup

package_name = 'my_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'pyrealsense2',
                      'opencv-python',
                      'numpy',],
    zip_safe=True,
    maintainer='yt6',
    maintainer_email='yt6@todo.todo',
    description='Realsense Circle Detector',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_circles = my_vision_pkg.detect_circles_node:main',
        ],
    },
)
