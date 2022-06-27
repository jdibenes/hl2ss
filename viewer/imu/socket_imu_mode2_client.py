
import socket
import numpy as np
import struct

HOST = "192.168.1.15"

# 3806 Accelerometer
# 3807 Gyroscope
# 3808 Magnetometer
PORT = 3808

# 
mode = 2

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    data = bytearray()
    state = 0

    s.connect((HOST, PORT))
    s.send(struct.pack('<B', mode))
    
    chunk = s.recv(64)
    if (len(chunk) == 0):
        print('no data')
    data.extend(chunk)

    extrinsics = np.frombuffer(chunk, dtype=np.float32).reshape(4,4)
    print(extrinsics)

