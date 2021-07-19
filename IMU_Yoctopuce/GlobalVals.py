from threading import Lock

host = '127.0.0.1'
port = 5003
socketTimeout = 600000

fileName = 'IMU_Yoctopuce.txt'
sysID = 0


DATA_BUFFER = []

NEW_DATA = False
END_IMU_SOCKET = False

DATA_BUFFER_MUTEX = Lock()
NEW_DATA_MUTEX = Lock()
END_IMU_SOCKET_MUTEX = Lock()
