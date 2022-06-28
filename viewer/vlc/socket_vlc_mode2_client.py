# OK

import socket
import numpy as np
import av
import struct

# hololens 2 address
HOST = "192.168.1.15"

# 3800 left front
# 3801 left left
# 3802 right front
# 3803 right right
PORT = 3800

# camera parameters (ignored, always 640x480 30 FPS)
width     = 640
height    = 480
framerate = 30

# encoding profiles
# 0: h264 base
# 1: h264 main
# 2: h264 high
# 3: h265 main (HEVC)
profile = 3

# encoded stream average bits per second (must be > 0)
bitrate = 1*1024*1024

# operating mode
# 0: video
# 1: video+pose
# 2: calibration (single transfer 640*480*2 + 16 floats (2,457,664 bytes))
mode = 2

# -----------------------------------------------------------------------------

if (profile == 3):
    codec_name = 'hevc'
else:
    codec_name = 'h264'

codec = av.CodecContext.create(codec_name, 'r')
tmpbuffer = bytearray()
tmpstate = 0
lastcaptime = None
frames = 0
total_bytes = (640*480*2 + 16)*4
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

lut_size = 640*480
floats = np.frombuffer(tmpbuffer, dtype=np.float32)

uv2x       = floats[:lut_size].reshape((480, 640))
uv2y       = floats[lut_size:(2*lut_size)].reshape((480, 640))
extrinsics = floats[(2*lut_size):].reshape((4,4))

print(extrinsics)
