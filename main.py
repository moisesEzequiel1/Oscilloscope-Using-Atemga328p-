

import struct
import serial 
import time


con = serial.Serial("COM3", 115200, timeout=1)
time.sleep(2)
data = [0]*1024


con.write(b'r')

rawdata = con.read(1024)
data = list(struct.unpack('B'*1024,rawdata))
data = [(float(x)/255.0 - 0.5) * 20 for x in data]
print(data)
