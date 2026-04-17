import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gunwoong/workspace/pcd2scenegraph/install/robot_simulation'
