# OK

import socket
import cv2
import numpy as np
import av
import time
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
# 1: video + rig pose
# 2: calibration (single transfer 640*480*2 + 16 floats (2,457,664 bytes))
mode = 0

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
chunk_size = 1024 # critical value

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.send(struct.pack('<BHHBBI', 0, width, height, framerate, profile, bitrate))

    while True:
        chunk = s.recv(chunk_size) 
        if (len(chunk) == 0):
            break
        tmpbuffer.extend(chunk)

        if (tmpstate == 0):
            if (len(tmpbuffer) >= 12):
                header = struct.unpack('<QI', tmpbuffer[:12])
                timestamp = header[0]
                payload_length = header[1]
                packetlen = 12 + payload_length
                tmpstate = 1
            continue
        elif (tmpstate == 1):
            if (len(tmpbuffer) >= packetlen):
                payload = tmpbuffer[12:packetlen]
                tmpbuffer = tmpbuffer[packetlen:]
                tmpstate = 0
            else:
                continue

        packets = codec.parse(payload)

        for packet in packets:
            for frame in codec.decode(packet):
                img = frame.to_ndarray(format='bgr24')

                frames += 1
                if (not lastcaptime):
                    lastcaptime = time.perf_counter()
                elif (frames >= 60):
                    print('FPS: {}'.format(frames/(time.perf_counter() - lastcaptime)))
                    lastcaptime = time.perf_counter()
                    frames = 0
                    
                cv2.imshow('video', img)
                cv2.waitKey(1)
