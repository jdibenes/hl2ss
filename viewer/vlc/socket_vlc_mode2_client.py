
import socket
import cv2
import numpy as np
import av
import time
import struct

HOST = "192.168.1.15"

# 3800 VLC0
# 3801 VLC1
# 3802 VLC2
# 3803 VLC3
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
# 2: calibration
mode = 2

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    if (profile == 3):
        codec_name = 'hevc'
    else:
        codec_name = 'h264'

    codec = av.CodecContext.create(codec_name, 'r')
    tmpbuffer = bytearray()
    tmpstate = 0
    lastcaptime = None
    frames = 0

    totalbytes = 640*480*4*2 + 64
    nextfragment = 1024

    s.connect((HOST, PORT))
    s.send(struct.pack('<B', mode))

    while totalbytes > 0:
        chunk = s.recv(nextfragment)
        if (len(chunk) == 0): break
        tmpbuffer.extend(chunk)
        totalbytes -= len(chunk)
        if (totalbytes < nextfragment):
            nextfragment = totalbytes

    print(totalbytes)
    print(len(tmpbuffer))

    uv2x = np.frombuffer(tmpbuffer[:(640*480*4)], dtype=np.float32).reshape((480,640))
    uv2y = np.frombuffer(tmpbuffer[(640*480*4):(640*480*4*2)], dtype=np.float32).reshape((480,640))
    extrinsics = np.frombuffer(tmpbuffer[(640*480*4*2):(640*480*4*2+64)], dtype=np.float32).reshape(4,4)

    print(extrinsics)

