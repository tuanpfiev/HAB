import numpy as np
import datetime
EXPERIMENT_TIME = datetime.datetime(2021,5,16,15,15,0).timestamp()

N_BALLOON = 5
N_REAL_BALLOON = 3
REAL_BALLOON = np.array([1,2,3])
ALL_BALLOON = np.array([1,2,3,4,5])
LORA_PAIR_NUM = np.array([1,2,3])
# GPS-Loggers.py
GPS_READER_SOCKET = 5001

# GPS_ALL_DISTRO = [5200,5210,5290,5280, 5270]
GPS_ALL_DISTRO = [5200,5210,5290, 5280]
IMU_READER_SOCKET = 5003
IMU_DISTRO_SOCKET = 5004
RSSI_ALLOCATION_DISTRO_SOCKET = [5185, 5195]


# LoRa-Radio-RSSI.py
LORA_GPS_RECEIVE_SOCKET = [GPS_ALL_DISTRO[0], GPS_ALL_DISTRO[1]]
LORA_RSSI_DISTRO_SOCKET = np.array([    [5100, 5110],       # EKF
                                        [5190, 5191]])      # RFD900 Network

RSSI_LOGGER_SOCKET = LORA_RSSI_DISTRO_SOCKET[1]

RSSI_DISTRO_SOCKET = [5120] 
LORA_RSSI_CALIBRATION_SIZE = 10

EKF_GPS_RECEIVE_SOCKET = GPS_ALL_DISTRO[2]
EKF_IMU_RECEIVE_SOCKET = IMU_READER_SOCKET
EKF_RSSI_RECEIVE_SOCKET = LORA_RSSI_DISTRO_SOCKET[0]
EKF_OUTPUT_DISTRO_SOCKET = 5500
EKF_OUTPUT_RECEIVE_SOCKET = EKF_OUTPUT_DISTRO_SOCKET

EKF_ALL_DISTRO_SOCKET = [5510]
LA_EKF_ALL_RECEIVE_SOCKET = EKF_ALL_DISTRO_SOCKET[0]

EKF_LOOPTIME = 0.02
EKF_RSSI_CALIBRATION_SIZE = 10
EKF_ANCHOR = np.array([1,2,3,4,5])

# Localisation-RSSI.py
LA_ITERATION = 50
LA_R_RSSI_RECEIVE_SOCKET = RSSI_DISTRO_SOCKET[0]
LA_R_GPS_RECEIVE_SOCKET = GPS_ALL_DISTRO[3]
LA_ANCHOR = np.array([2,3,4])
LA_LEADER = 5
LA_LOOP_TIME = 50
SIGMA_RSSI_RANGE = 50

# Localisation-RSSI-OnlyGPS.py
LA_G_GPS_RECEIVE_SOCKET = 5270

# temperatureSensor.py
TS_TEMPT_DISTRO_SOCKET = 5600



TEMP_LOGGER_SOCKET = 5600


LAT_REF = -37.62397
LON_REF = 145.12325
ALT_REF = 100


