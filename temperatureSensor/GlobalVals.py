from threading import Lock
import sys
sys.path.insert(1,'../utils/')
import GlobalVariables

HOST = '127.0.0.1'
PORT_TEMPERATURE = GlobalVariables.TS_TEMPT_DISTRO_SOCKET
socketTimeout = 36000

fileName = 'TemperatureYoctopuce.txt'
sysID = 1

tempDataBuffer = []
timeDataBuffer = []

newTempData = False
endTempSocket = False

endTempSocketMutex = Lock()
newTempDataMutex = Lock()
appendTempDataMutex = Lock()
