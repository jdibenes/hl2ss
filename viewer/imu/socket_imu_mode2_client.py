# OK

import socket
import numpy as np
import struct

# hololens 2 address
HOST = "192.168.1.15"

# 3806 accelerometer
# 3807 gyroscope
# 3808 magnetometer (does not support mode 2)
PORT = 3806

# operating mode
# 0: samples
# 1: samples + rig pose
# 2: query extrinsics (single transfer 16 floats (64 bytes))
mode = 2 

#------------------------------------------------------------------------------

data = bytearray()
total_bytes = 64
chunk_size = 64

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.send(struct.pack('<B', mode))

    while total_bytes > 0:
        chunk = s.recv(chunk_size)
        size = len(chunk)
        if (size == 0):
            break
        data.extend(chunk)
        total_bytes -= size
        if (total_bytes < chunk_size):
            chunk_size = total_bytes

if (total_bytes > 0):
    print('Failed to obtain extrinsics')
    quit()

extrinsics = np.frombuffer(chunk, dtype=np.float32).reshape(4,4)
print(extrinsics)
