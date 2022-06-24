
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

    s.connect((HOST, PORT))
    s.send(struct.pack('<HHBBI', width, height, framerate, profile, bitrate))

    while True:
        chunk = s.recv(1024) # critical value: too high = lag, too low = lag
        if (len(chunk) == 0): break
        tmpbuffer.extend(chunk)

        if (tmpstate == 0):
            if (len(tmpbuffer) >= 12):
                header = struct.unpack('<QI', tmpbuffer[:12])
                timestamp = header[0]
                h264len = header[1]
                packetlen = 12 + h264len
                tmpstate = 1
            continue
        elif (tmpstate == 1):
            if (len(tmpbuffer) >= packetlen):
                h264payload = tmpbuffer[12:packetlen]
                tmpbuffer = tmpbuffer[packetlen:]
                tmpstate = 0
            else:
                continue

        packets = codec.parse(h264payload)

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
