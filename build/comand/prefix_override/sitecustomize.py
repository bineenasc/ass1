import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bob5t0nl0v3r/Documents/M.IA/a1/s2/TRI/ass1/install/comand'
