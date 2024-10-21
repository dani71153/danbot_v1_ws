import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/daniel/danbot_v1_ws/install/cinematica_v1'
