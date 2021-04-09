import numpy as np
def dataTruncate(dataList,nData):
    if len(dataList)==0 or len(dataList)<nData:
        return dataList
    
    interval = int(len(dataList)/nData)
    selectedIndex = np.arange(0,len(dataList),interval)
    result = []
    for i in range(len(selectedIndex)):
        result.append(dataList[selectedIndex[i]])
    
    # add the last element:
    result.append(dataList[-1])
    return result

class Dump:
    def __init__(self,sysID,x,y):
        self.sysID= sysID
        self.x = x
        self.y = y

# a = np.array([])

a = [Dump(0,0,0),Dump(0,1,1),Dump(0,2,2),Dump(0,3,3),Dump(0,4,4),Dump(0,5,5),Dump(0,6,6)]

print(a)

a = dataTruncate(a,100)
print(a)