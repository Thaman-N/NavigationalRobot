import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thaman/ethical_robot_ws/install/ethical_robot_sim'
