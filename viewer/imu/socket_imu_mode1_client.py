# OK

import socket
import numpy as np
import struct

# hololens 2 address
HOST = "192.168.1.15"

# 3806 accelerometer
# 3807 gyroscope
# 3808 magnetometer
PORT = 3806

# operating mode
# 0: samples
# 1: samples + rig pose
# 2: query extrinsics (single transfer 16 floats (64 bytes))
mode = 1

#------------------------------------------------------------------------------

data = bytearray()
state = 0
chunk_size = 1024

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.send(struct.pack('<B', mode))
    
    while True:
        chunk = s.recv(chunk_size) 
        if (len(chunk) == 0): break
        data.extend(chunk)

        if (state == 0):
            if (len(data) >= 12):
                header = struct.unpack('<QI', data[:12])
                timestamp = header[0]
                payload_length = header[1]
                packetlen = 12 + payload_length + 64
                state = 1
        elif (state == 1):
            if (len(data) >= packetlen):
                batch_end = packetlen - 64
                batch = data[12:batch_end]
                pose = np.frombuffer(data[batch_end:packetlen], dtype=np.float32).reshape((4,4))
                data = data[packetlen:]
                firstinbatch = struct.unpack('<Qfff', batch[:20])
                print(firstinbatch[1:4])
                print(pose)
                state = 0
