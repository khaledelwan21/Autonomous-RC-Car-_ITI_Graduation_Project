import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/khaled-ahmed-elwan/DATA/car_robot_ws/install/ps4_robot_control'
