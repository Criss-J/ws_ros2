import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yangdrone/ws_ros2/install/my_offboard_ctrl'
