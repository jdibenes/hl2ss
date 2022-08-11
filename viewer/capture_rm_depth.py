#------------------------------------------------------------------------------
# rm depth lt capture test 
#------------------------------------------------------------------------------

import hl2ss 
import cv2
import os

# Settings --------------------------------------------------------------------

host = "192.168.1.15"
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW
name = 'lt'
chunk_size = 4096
mode = hl2ss.StreamMode.MODE_1
brightness = 8
frames = hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS*5
path = os.path.join('.', 'data')

#------------------------------------------------------------------------------

wr_frames = 0
rd_frames = 0

wr = hl2ss.wr_rm_depth(path, name, mode)
rx = hl2ss.rx_rm_depth(host, port, chunk_size, mode)

wr.open()
rx.open()

while (wr_frames < frames):
    data = rx.get_next_packet()
    wr.write(data)

    cv2.imshow('depth', data.payload.depth*brightness)
    cv2.imshow('ab', data.payload.ab*brightness)
    cv2.waitKey(1)

    wr_frames += 1

rx.close()
wr.close()

ca = hl2ss.continuity_analyzer(hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS * hl2ss.Parameters_RM_DEPTH_LONGTHROW.PERIOD)
rd = hl2ss.rd_rm_depth(path, name)
rd.open()

while (True):
    data = rd.read()
    if (data is None):
        break

    print(data.timestamp)
    print(data.pose)

    cv2.imshow('depth', data.payload.depth*brightness)
    cv2.imshow('ab', data.payload.ab*brightness)
    cv2.waitKey(1000 // hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)

    rd_frames += 1

    state = ca.push(data.timestamp)
    if (state != 0):
        raise Exception('discontinuous frame')

rd.close()

print('written frames {v}'.format(v=wr_frames))
print('read frames {v}'.format(v=rd_frames))
