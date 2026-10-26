import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sw/DevSpace/FMS/simulation/ros_ws/install/pinky_llm'
