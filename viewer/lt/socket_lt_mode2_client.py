# OK

import socket
import numpy as np
import av
import struct

# hololens 2 address
HOST = "192.168.1.15"
PORT = 3805

# operating mode
# 0: video
# 1: video+pose
# 2: calibration (single transfer 320*288*2 + 16 + 1 floats (184,337 bytes))
mode = 2

# -----------------------------------------------------------------------------

tmpbuffer = bytearray()
total_bytes = (320*288*2 + 16 + 1)*4
chunk_size = 1024

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:  
    s.connect((HOST, PORT))
    s.send(struct.pack('<B', mode))

    while total_bytes > 0:
        chunk = s.recv(chunk_size)
        size = len(chunk)
        if (size == 0):
             break
        tmpbuffer.extend(chunk)
        total_bytes -= size
        if (total_bytes < chunk_size):
            chunk_size = total_bytes

if (total_bytes > 0):
    print('Failed to obtain calibration data')
    quit()

lut_size = 320*288
floats = np.frombuffer(tmpbuffer, dtype=np.float32)

uv2x       = floats[:lut_size].reshape((288, 320))
uv2y       = floats[lut_size:(2*lut_size)].reshape((288, 320))
extrinsics = floats[(2*lut_size):(2*lut_size+16)].reshape((4,4))
scale      = floats[(2*lut_size+16):]

print(extrinsics)
print(scale)
