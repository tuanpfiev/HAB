from threading import Lock
import numpy as np
import sys

sys.path.insert(1,'../utils/')
from common_class import *


stream_name = 'RMITballoon_Data'
N_BALLOON = 4
GPS_ALL = np.array([GPS()]*N_BALLOON)
