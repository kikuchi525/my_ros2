from setuptools import find_packages, setup

package_name = 'my_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['my_vision_pkg', 'my_vision_pkg.*']),
    package_data={
    'my_vision_pkg': ['msg/*.py'],
},
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
            'detect_circles_1shot = my_vision_pkg.detect_circles_node2:main',
            'circle_subscriber = my_vision_pkg.circle_subscriber_no_msg:main',
            
        ],
    },
)
