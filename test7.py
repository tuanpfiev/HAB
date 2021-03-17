import socket, time
while True:
    try: 
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((localhost,5003))
        while True:
            data, addr = sock.recvfrom(1024)
            print(data)
    except:
        print('Nothing')
        time.sleep(1)