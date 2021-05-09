from threading import Lock

HOST = '127.0.0.1'
PORT_TEMPERATURE = 5600
socketTimeout = 60

fileName = 'TemperatureYoctopuce.txt'
sysID = 1

tempDataBuffer = []
timeDataBuffer = []

newTempData = False
endTempSocket = False

endTempSocketMutex = Lock()
newTempDataMutex = Lock()
appendTempDataMutex = Lock()
