import numpy as np 

class IMU:
    def __init__(self, sysID = None, raw_qt = None, mag_vector = None, accel = None, gyros = None, epoch = None, euler = None, gyro = None):
        self.sysID = sysID if sysID is not None else 0
        self.raw_qt = raw_qt if raw_qt is not None else np.array([[0.0, 0.0, 1.0, 0.0]]).T
        self.mag_vector = mag_vector if mag_vector is not None else np.zeros([3,1])
        self.accel = accel if accel is not None else np.array([[0.0, 0.0, -9.81]]).T
        self.gyros = gyros if gyros is not None else np.zeros([3,1])
        self.epoch = epoch if epoch is not None else 0.0
        self.euler = euler if euler is not None else np.zeros([3,1])
        self.gyro = gyro if gyro is not None else np.zeros([3,1])

class GPS:
    def __init__(self, sysID = None, lat = None, lon = None, alt = None, epoch = None):
        self.sysID = sysID if sysID is not None else 0
        self.lat = lat if lat is not None else 0.0
        self.lon = lon if lon is not None else 0.0
        self.alt = alt if alt is not None else 0.0
        self.epoch = epoch if epoch is not None else 0.0

class RSSI:
    def __init__(self, rssi_filtered = None, distance = None, epoch = None):
        self.rssi_filtered = rssi_filtered if rssi_filtered is not None else 0.01
        self.distance = distance if distance is not None else 0.01
        self.epoch = epoch if epoch is not None else 0.0

class POS_XYZ:
    def __init__(self, x = None, y = None, z = None):
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.z = z if z is not None else 0.0

