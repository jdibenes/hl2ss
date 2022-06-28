# OK

import socket
import cv2
import numpy as np
import av
import time
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
profile = 3

# encoded stream average bits per second (must be > 0)
bitrate = 5*1024*1024

# operating mode
# 0: video
# 1: video + camera pose
# 2: query intrinsics (single transfer 25 floats (100 bytes))
mode = 1

#------------------------------------------------------------------------------

if (profile == 3):
    codec_name = 'hevc'
else:
    codec_name = 'h264'

codec = av.CodecContext.create(codec_name, 'r')
tmpbuffer = bytearray()
tmpstate = 0
lastcaptime = None
frames = 0
chunk_size = 8192 # critical value

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:   
    s.connect((HOST, PORT))
    s.send(struct.pack('<BHHBBI', mode, width, height, framerate, profile, bitrate))

    while True:
        chunk = s.recv(chunk_size) 
        if (len(chunk) == 0): break
        tmpbuffer.extend(chunk)

        if (tmpstate == 0):
            if (len(tmpbuffer) >= 12):
                header = struct.unpack('<QI', tmpbuffer[:12])
                timestamp = header[0]
                payload_length = header[1]
                packetlen = 12 + payload_length + 64
                tmpstate = 1
            continue
        elif (tmpstate == 1):
            if (len(tmpbuffer) >= packetlen):
                payload_end = packetlen - 64
                payload = tmpbuffer[12:payload_end]
                pose = tmpbuffer[payload_end:packetlen]
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
                    pose_matrix = np.frombuffer(pose, dtype=np.float32).reshape((4,4))
                    print(pose_matrix)

                cv2.imshow('video', img)
                cv2.waitKey(1)
