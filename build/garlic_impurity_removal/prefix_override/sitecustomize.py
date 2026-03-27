import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sujit/Documents/garlic_ws/install/garlic_impurity_removal'
