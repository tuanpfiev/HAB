from threading import Lock

host = '127.0.0.1'
port = 5013
socketTimeout = 60

fileName = 'TemperatureYoctopuce.txt'


tempDataBuffer = []
timeDataBuffer = []

newTempData = False
endTempSocket = False

endTempSocketMutex = Lock()
newTempDataMutex = Lock()
appendTempDataMutex = Lock()
