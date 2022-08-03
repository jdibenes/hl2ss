#------------------------------------------------------------------------------
# This script receives video frames and hand tracking data from the HoloLens.
# Then, it projects the latest received hand joint positions onto the latest
# video frame. First, the intrinsics of the camera are obtained from a mode 2
# transfer. Then, the video stream is set to mode 1 to include the camera pose.
# The camera pose of the frame is used to project the hand joints. The streams
# are independent and run at different frame rates. Hand tracking data is sent
# at 60 Hz and video frames are sent at 30 Hz. Due to the different latencies
# of the streams and the lazy synchronization used in this implementation, the 
# video lags behind hand motion. As future work, the timestamps of both streams
# should be used to generate proper composite frames.
#------------------------------------------------------------------------------

import hl2ss
import numpy as np
import av
import threading
import cv2

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = "192.168.1.15"

# Ports
port_pv = hl2ss.StreamPort.PERSONAL_VIDEO
port_si = hl2ss.StreamPort.SPATIAL_INPUT

# Camera parameters
# See etc/hl2_capture_formats.txt for a list of supported formats
width     = 1920
height    = 1080
framerate = 30

# Video Encoding profiles
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 5*1024*1024

#------------------------------------------------------------------------------
# Connect in mode 2 to query camera intrinsics

data = hl2ss.download_calibration_pv(host, port_pv, width, height, framerate, profile, bitrate)
K = data.projection

#------------------------------------------------------------------------------
# Open PV stream in mode 1 to obtain camera poses and open SI stream to obtain
# hand tracking data

enable     = True
last_frame = None
last_pose  = None
last_left  = None
last_right = None

def recv_pv():
    global enable
    global last_frame
    global last_pose

    codec_h264 = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')
    client = hl2ss.connect_client_pv(host, port_pv, 8192, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate)

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

    client = hl2ss.connect_client_si(host, port_si, 2048)

    while (enable):
        data = client.get_next_packet()
        si = hl2ss.unpack_si(data.payload)

        if (si.is_valid_hand_left()):
            last_left = si.get_hand_left()
        else:
            last_left = None

        if (si.is_valid_hand_right()):
            last_right = si.get_hand_right()
        else:
            last_right = None

    client.close()

thread_pv = threading.Thread(target=recv_pv)
thread_si = threading.Thread(target=recv_si)

thread_pv.start()
thread_si.start()

def render_hand(image, hand, P):
    for joint in range(0, hl2ss.HandJointKind.TOTAL):
        pose = hand.get_joint_pose(joint)
        point = pose.position.reshape((1,3))
        point = np.concatenate([point, np.array([1]).reshape((1,1))], axis=1)
        pixel = np.matmul(point, P)
        pixel = pixel / pixel[0,2]
        image = cv2.circle(image, (int(pixel[0,0]), (int(pixel[0,1]))), 5, (0, 0, 255), 3)
    return image

try:
    while True:
        if (last_frame is None):
            continue

        next_frame = last_frame
        next_pose  = last_pose
        next_left  = last_left
        next_right = last_right

        image = next_frame.to_ndarray(format='bgra')
        P = np.matmul(np.linalg.inv(next_pose), K)

        if (next_left is not None):
            image = render_hand(image, next_left, P)

        if (next_right is not None):
            image = render_hand(image, next_right, P)

        cv2.imshow('video', image)
        cv2.waitKey(1)
except:
    pass

enable = False
thread_pv.join()
thread_si.join()
