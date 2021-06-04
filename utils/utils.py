# ==================================================
# JETSON USB port diagram (side view)
#------------------------------------
#       |  USB 1  |   USB 3
#  HDMI |  USB 2  |   USB 4
# -----------------------------------
# Lora must be plugged into USB 3
# GPS must be pugged into USB 4
# RFD900 must be plugged into USB 2
# 
# in ANY plugging order
# ==================================================


import serial
import time
import subprocess


def run_command(command):
    p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)    
    return iter(p.stdout.readline,b'')


def get_port(device_name):

    if device_name == 'Lora1':
        attach_str = ['usb 1-2.1.1: pl2303 converter now attached to']

    elif device_name == 'Lora2':
        attach_str = ['usb 1-2.1.2: pl2303 converter now attached to']
    
    elif device_name == 'GPS':
        attach_str = ['usb 1-2.1.3: pl2303 converter now attached to']

    elif device_name == 'RFD900':
        attach_str = ['FTDI USB Serial Device converter now attached to']  # USB 2

    else:
        print(device_name + ' was not attached')

    command = 'dmesg'    

    port = ''
    for line in run_command(command):
        tmp = line.decode('utf-8')
        for i in range(len(attach_str)):
            if attach_str[i] in tmp:
                index = tmp.find('ttyUSB')
                port = tmp[index:-1]

    return '/dev/'+port
