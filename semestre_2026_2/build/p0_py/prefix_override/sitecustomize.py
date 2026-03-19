import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arrgusr/ROS2Dev/clss_Robotica/semestre_2026_2/install/p0_py'
