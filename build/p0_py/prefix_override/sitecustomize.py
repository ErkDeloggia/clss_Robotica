import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arrgusr/ROS2Dev/clss_Robotica/install/p0_py'
