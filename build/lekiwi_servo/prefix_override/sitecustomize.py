import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/acy/lekiwi/lekiwi_ws/install/lekiwi_servo'
