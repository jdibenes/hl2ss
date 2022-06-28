# OK

import socket
import numpy as np
import struct

# hololens 2 address
HOST = "192.168.1.15"

# pv port
PORT = 3810

# camera parameters (see pv_list.txt for supported formats)
width     = 1920
height    = 1080
framerate = 30

# encoding profiles
# 0: h264 base
# 1: h264 main
# 2: h264 high
# 3: h265 main (HEVC)
profile = 0

# encoded stream average bits per second (must be > 0)
bitrate = 5*1024*1024

# operating mode
# 0: video
# 1: video + camera pose
# 2: query intrinsics (single transfer 25 floats (100 bytes))
mode = 2

#------------------------------------------------------------------------------

total_bytes = 100
chunk_size = 100
data = bytearray()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.send(struct.pack('<BHHBBI', mode, width, height, framerate, profile, bitrate))

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
    print('Failed to obtain intrinsics')
    quit()

intrinsics = np.frombuffer(data, dtype=np.float32)

focal_length                     = intrinsics[0:2]
principal_point                  = intrinsics[2:4]
radial_distortion                = intrinsics[4:7]
tangential_distortion            = intrinsics[7:9]
undistorted_projection_transform = intrinsics[9:].reshape((4,4))

print('focal length')
print(focal_length)

print('principal point')
print(principal_point)

print('radial distortion')
print(radial_distortion)

print('tangential distortion')
print(tangential_distortion)

print('undistorted projection transform')
print(undistorted_projection_transform)
