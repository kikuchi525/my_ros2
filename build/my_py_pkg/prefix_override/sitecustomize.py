import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tabatakenta/kikuchi_jikken/my_ros2/install/my_py_pkg'
