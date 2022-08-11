#------------------------------------------------------------------------------
# pv capture test
#------------------------------------------------------------------------------

import hl2ss
import cv2
import os

# Settings --------------------------------------------------------------------

host = "192.168.1.15"
port = hl2ss.StreamPort.PERSONAL_VIDEO
mode = hl2ss.StreamMode.MODE_1
chunk_size = hl2ss.ChunkSize.PERSONAL_VIDEO
width = 1280
height = 720
framerate = 30
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = 5*1024*1024
frame_format = 'bgr24'
frames = framerate*5
path = os.path.join('.', 'data')

#------------------------------------------------------------------------------

wr_frames = 0
rd_frames = 0

rx = hl2ss.rx_pv(host, port, chunk_size, mode, width, height, framerate, profile, bitrate, frame_format)
wr = hl2ss.wr_pv(path, mode, width, height, framerate, 'mpeg4', 8*1024*1024, frame_format)

wr.open()
rx.open()

while (wr_frames < frames):
    data = rx.get_next_packet()
    wr.write(data)

    cv2.imshow('Video', data.payload)
    cv2.waitKey(1)

    wr_frames += 1

rx.close()
wr.close()

ca = hl2ss.continuity_analyzer(hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS * 1/30)
rd = hl2ss.rd_pv(path, frame_format)
rd.open()

while (True):
    data = rd.read()
    if (data is None):
        break

    print(data.timestamp)
    print(data.pose)

    cv2.imshow('Video', data.payload)
    cv2.waitKey(1000 // framerate)

    rd_frames += 1

    state = ca.push(data.timestamp)
    if (state != 0):
        raise Exception('discontinuous frame')

rd.close()

print('written frames {v}'.format(v=wr_frames))
print('read frames {v}'.format(v=rd_frames))
