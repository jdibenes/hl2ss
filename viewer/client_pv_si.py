# OK

# This script receives video frames from the RGB camera and Hand Tracking data
# Then, it projects the latest observed hand joints onto the lastest video frame
# The intrinsics of the RGB camera are obtained by first connecting in mode 2
# The pose of the RGB camera is embeded in the video stream (mode 1) which is
# unpacked and used to project the hand joints
# The streams are independent and run at different frame rates
# Hand tracking data is sent at 60 Hz and video frames are sent at 30 Hz
# Phase relationship is unknown
# Due to the different latencies of the streams and the lazy "synchronization"
# used in this implementation, the video lags behind hand motion
# A proper synchronization scheme should use the timestamps of both streams to
# generate proper composite frames

import hl2ss
import socket
import numpy as np
import struct
import av
import threading
import cv2

# hololens 2 address
host = "192.168.1.15"

# pv port
port_pv = 3810
port_si = 3812

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

#------------------------------------------------------------------------------
# Connect in mode 2 to query camera intrinsics

data = hl2ss.get_mode2_pv(host, port_pv, width, height, framerate, profile, bitrate)

focal_length          = data.focal_length
principal_point       = data.principal_point
radial_distortion     = data.radial_distortion
tangential_distortion = data.tangential_distortion
projection            = data.projection

print('got intrinsics')

#------------------------------------------------------------------------------
# Open PV stream in mode 1 to obtain the camera pose and open SI stream to
# to obtain hand tracking data

enable = True
frame_period = 33

last_frame = None
last_pose = None
last_left = None
last_right = None


def recv_pv():
    global enable
    global last_frame
    global last_pose

    chunk_size = 8192
    mode = hl2ss.StreamMode.MODE_1
    client = hl2ss.gatherer()
    codec_h264 = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')

    client.open(host, hl2ss.StreamPort.PERSONAL_VIDEO, chunk_size, mode)
    client.configure(hl2ss.create_configuration_for_video(mode, width, height, framerate, profile, bitrate))

    while (enable):
        data = client.get_next_packet()
        last_pose = data.pose
        packets = codec_h264.parse(data.payload)
        for packet in packets:
            for frame in codec_h264.decode(packet):
                last_frame = frame

    client.close()

def recv_si():
    global enable
    global last_left
    global last_right

    base_left =  1 + 36 + 24
    base_right = base_left + 26*36
    hand_end = base_right + 26*36

    client = hl2ss.gatherer()

    client.open(host, hl2ss.StreamPort.SPATIAL_INPUT, 2048, hl2ss.StreamMode.MODE_0)

    while (enable):
        data = client.get_next_packet()
        data_si = data.payload

        valid = struct.unpack('<B', data_si[:1])
        valid = valid[0]
        valid_left = (valid & 0x04) != 0
        valid_right = (valid & 0x08) != 0                
            
        if (valid_left):
            last_left = data_si[base_left:base_right]
        else:
            last_left = None

        if (valid_right):
            last_right = data_si[base_right:hand_end]
        else:
            last_right = None

threading.Thread(target=recv_pv).start()
threading.Thread(target=recv_si).start()

while (last_frame is None):
    pass

def render_hand(image, next_hand, cam, K, width):
    for i in range(0, 26):
        base = i*36
        pos = np.frombuffer(next_hand[(base+16):(base+16+12)], dtype=np.float32).reshape((1,3))
        posh = np.concatenate([pos, np.array([1]).reshape((1,1))], axis=1)    
        pos3d = np.matmul(posh, cam)
        pos3d = pos3d / pos3d[0,3]
        posp = np.matmul(pos3d[0, :3], K)
        posp = posp / posp[2]
        image = cv2.circle(image, (width - int(posp[0]), int(posp[1])), 5, (0, 0, 255), 3)
    return image


K = np.array([[focal_length[0], 0, principal_point[0]],
                [0, focal_length[1], principal_point[1]],
                [0, 0, 1]]).transpose()

while (enable):
    next_frame = last_frame
    next_pose = last_pose
    next_left = last_left
    next_right = last_right

    image = next_frame.to_ndarray(format='bgra')
    cam = np.linalg.inv(np.frombuffer(next_pose, dtype=np.float32).reshape((4,4)))

    if (next_left is not None):
        image = render_hand(image, next_left, cam, K, width)

    if (next_right is not None):
        image = render_hand(image, next_right, cam, K, width)
            
    cv2.imshow('video', image)
    cv2.waitKey(1)
