import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/wwn-0x50026b738078fa8c-part2/RISC_induction/line_follower_ws_/install/ros2_line_follower'
