import sys
sys.path.insert(0, '/home/ros/dev_ws/install/competition_bringup/lib/python3.10/site-packages')
import os
os.environ.setdefault('PYTHONUNBUFFERED', '1')
from competition_bringup.referee_node import main
main()
