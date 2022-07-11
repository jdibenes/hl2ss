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

import socket
import numpy as np
import struct
import av
import threading
import cv2

# hololens 2 address
HOST = "192.168.1.15"

# pv port
PV_PORT = 3810
SI_PORT = 3812

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

total_bytes = 100
chunk_size = 100
data = bytearray()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PV_PORT))
    s.send(struct.pack('<BHHBBI', 2, width, height, framerate, profile, bitrate))

    while total_bytes > 0:
        chunk = s.recv(chunk_size)
        size = len(chunk)
        if (size == 0):
            break
        data.extend(chunk)
        total_bytes -= size
        if (total_bytes < chunk_size):
            chunk_size = total_bytes

if (total_bytes > 0):
    print('Failed to obtain intrinsics')
    quit()

intrinsics = np.frombuffer(data, dtype=np.float32)

focal_length                     = intrinsics[0:2]
principal_point                  = intrinsics[2:4]
radial_distortion                = intrinsics[4:7]
tangential_distortion            = intrinsics[7:9]
undistorted_projection_transform = intrinsics[9:].reshape((4,4))

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

# (ok, state, buffer, timestamp, size, payload, pose)
def unpack_pv_payload(state_tuple):
    ok = state_tuple[0]
    state = state_tuple[1]
    buffer = state_tuple[2]
    ts = state_tuple[3]
    size = state_tuple[4]
    payload = state_tuple[5]
    pose = state_tuple[6]

    ok = False
    if (state == 0):
        if (len(buffer) >= 12):
            header = struct.unpack('<QI', buffer[:12])
            ts = header[0]
            size = header[1] + 12 + 64
            state = 1
    elif (state == 1):
        if (len(buffer) >= size):
            payload = buffer[12:(size-64)]
            pose = buffer[(size-64):size]
            buffer = buffer[size:]
            state = 0
            ok = True

    return (ok, state, buffer, ts, size, payload, pose)


def recv_pv():
    global enable
    global last_frame
    global last_pose

    state_h264 = (False, 0, bytearray(), None, None, None, None)
    codec_h264 = av.CodecContext.create('hevc', 'r')

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket_h264:
        socket_h264.connect((HOST, PV_PORT))
        socket_h264.send(struct.pack('<BHHBBI', 1, width, height, framerate, profile, bitrate))

        while enable:
            chunk_h264 = socket_h264.recv(8192)
            if (len(chunk_h264) <= 0):
                print('pv disconnected')
                break
            state_h264[2].extend(chunk_h264)
            state_h264 = unpack_pv_payload(state_h264)

            while (state_h264[0]):
                last_pose = state_h264[6]
                packets = codec_h264.parse(state_h264[5])
                for packet in packets:
                    for frame in codec_h264.decode(packet):
                        last_frame = frame

                state_h264 = unpack_pv_payload(state_h264)


def recv_si():
    global enable
    global last_left
    global last_right

    packet_size = 8 + 1 + 36 + 24 + 2*26*36
    base_left = 8 + 1 + 36 + 24
    base_right = base_left + 26*36 
    data_si = bytearray()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket_si:
        socket_si.connect((HOST, SI_PORT))

        while enable:
            chunk_si = socket_si.recv(2048)
            if (len(chunk_si) <= 0):
                print('si disconnected')
                break
            data_si.extend(chunk_si)
            if (len(data_si) >= packet_size):
                valid = struct.unpack('<B', data_si[8:9])
                valid = valid[0]
                valid_left = (valid & 0x04) != 0
                valid_right = (valid & 0x08) != 0
                
                if (valid_left):
                    last_left = data_si[base_left:base_right]
                else:
                    last_left = None

                if (valid_right):
                    last_right = data_si[base_right:packet_size]
                else:
                    last_right = None

                data_si = data_si[packet_size:]

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
    cv2.waitKey(frame_period)
