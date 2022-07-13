# OK

import hl2ss
import cv2
import numpy as np
import av
import time

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port number
port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

# Operating mode
# 0: video
# 1: video + pose
# 2: calibration (single transfer 640*480*2 + 16 floats (2,457,664 bytes))
mode = hl2ss.StreamMode.MODE_1

# Camera parameters (ignored, always 640x480 30 fps)
width     = 640
height    = 480
framerate = 30

# Encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second (must be > 0)
bitrate = 1*1024*1024

# Maximum number of bytes to read from the socket buffer per step
# Use an appropriate power-of-two value
chunk_size = 1024

# Pose frequency
# Display pose every 'pose_frequency' frames if streaming in mode 1
pose_frequency = 30

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.get_mode2_rm_vlc(host, port)

    print(data.uv2xy.shape)
    print(data.extrinsics)

    quit()


client = hl2ss.gatherer()
codec = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')
frames = 0

client.open(host, port, chunk_size, mode)
client.configure(hl2ss.create_configuration_for_video(mode, width, height, framerate, profile, bitrate))

while True:
    data = client.get_next_packet()
    packets = codec.parse(data.payload)

    for packet in packets:
        for frame in codec.decode(packet):
            image = frame.to_ndarray(format='bgr24')
            frames += 1

            if (mode == hl2ss.StreamMode.MODE_1 and frames >= pose_frequency):
                frames = 0
                print('Pose at {timestamp}'.format(timestamp=data.timestamp))
                print(data.pose)

            cv2.imshow('video', image)
            cv2.waitKey(1)
