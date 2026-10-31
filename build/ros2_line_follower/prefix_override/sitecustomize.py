import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/arpan/SSD_EXT/RISC_induction/line_follower_ws_/install/ros2_line_follower'
