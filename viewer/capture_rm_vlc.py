#------------------------------------------------------------------------------
# rm vlc capture test
#------------------------------------------------------------------------------

import hl2ss
import cv2
import os

# Settings --------------------------------------------------------------------

host = "192.168.1.15"
port = hl2ss.StreamPort.RM_VLC_LEFTFRONT
name = 'lf'
chunk_size = hl2ss.ChunkSize.RM_VLC
mode = hl2ss.StreamMode.MODE_1
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = 1*1024*1024
frame_format = 'bgr24'
frames = hl2ss.Parameters_RM_VLC.FPS*5
path = os.path.join('.', 'data')

#------------------------------------------------------------------------------

wr_frames = 0
rd_frames = 0

rx = hl2ss.rx_rm_vlc(host, port, chunk_size, mode, profile, bitrate, frame_format)
wr = hl2ss.wr_rm_vlc(path, name, mode, 'mpeg4', 2*1024*1024, frame_format)

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

ca = hl2ss.continuity_analyzer(hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS * hl2ss.Parameters_RM_VLC.PERIOD)
rd = hl2ss.rd_rm_vlc(path, name, frame_format)
rd.open()

while (True):
    data = rd.read()
    if (data is None):
        break

    print(data.timestamp)
    print(data.pose)

    cv2.imshow('Video', data.payload)
    cv2.waitKey(1000 // hl2ss.Parameters_RM_VLC.FPS)

    rd_frames += 1

    state = ca.push(data.timestamp)
    if (state != 0):
        raise Exception('discontinuous frame')

rd.close()

print('written frames {v}'.format(v=wr_frames))
print('read frames {v}'.format(v=rd_frames))
