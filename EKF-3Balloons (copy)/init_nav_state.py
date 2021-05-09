import numpy as np
from utilities import *

def init_nav_state(u,m,pos):
    f = u
    initial_heading = np.arctan2(m[1],m[0])[0]
    roll = np.arctan2(-f[1],-f[2])[0]
    norm = np.linalg.norm(f[0:3])
    pitch = np.arctan2(f[0],norm)[0]
    Rb2t = Rt2b(np.array([roll, pitch, initial_heading])).T
    q = decm2q(Rb2t)
    x_h = np.zeros((10,1))
    x_h[-4:] = q
    x_h[:3] = pos
    angle  = np.zeros((3,1))
    angle[0] = np.rad2deg(roll)
    angle[1] = np.rad2deg(pitch)
    angle[2] = np.rad2deg(initial_heading)
    return x_h, angle




