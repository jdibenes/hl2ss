
import socket
import numpy as np
import struct

HOST = "192.168.1.15"

# 3806 Accelerometer
# 3807 Gyroscope
# 3808 Magnetometer
PORT = 3806

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    data = bytearray()
    state = 0

    s.connect((HOST, PORT))
    
    while True:
        chunk = s.recv(1024) 
        if (len(chunk) == 0): break
        data.extend(chunk)

        if (state == 0):
            if (len(data) >= 12):
                header = struct.unpack('<QI', data[:12])
                timestamp = header[0]
                accellen = header[1]
                packetlen = 12 + accellen
                state = 1
        elif (state == 1):
            if (len(data) >= packetlen):
                batch = data[12:packetlen]
                data = data[packetlen:]
                firstinbatch = struct.unpack('<Qfff', batch[:20])
                print(firstinbatch[1:4])
                state = 0
