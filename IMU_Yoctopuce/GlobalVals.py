from threading import Lock

host = '127.0.0.1'
port = 5013
socketTimeout = 600000

fileName = 'IMU_Yoctopuce.txt'
sysID = 0


tempDataBuffer = []
timeDataBuffer = []

newTempData = False
endTempSocket = False

endTempSocketMutex = Lock()
newTempDataMutex = Lock()
appendTempDataMutex = Lock()
