def sysID_to_index(sysID):
    if sysID == 1:
        return 1
    elif sysID == 2:
        return 2
    elif sysID == 253:
        return 3
    elif sysID == 254:
        return 4
    elif sysID == 255:
        return 5
    else:
        print('SysID should be in the range 1-5')
        os._exit(1)
        return 0

def extract_string_data(preString,endString,string_data):
    preIndex = string_data.find(preString)
    endIndex = string_data.find(endString, preIndex)
    return string_data[preIndex + len(preString):endIndex]

def convert_to_array(string_data):
    start_parsing = 0
    array = []
    while True:
        comma_index = string_data.find(",",start_parsing)
        if comma_index != -1:
            val = float(string_data[start_parsing:comma_index])
            array.append(val)
            start_parsing = comma_index + 1
        else:
            try:
                val = float(string_data[start_parsing:len(string_data)])
            except:
                break
            array.append(val)
            break
    return array