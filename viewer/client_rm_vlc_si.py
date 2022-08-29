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
import hl2ss_utilities
import threading
import cv2

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = "192.168.1.15"

# Port
port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

# Pinhole model path
model_path = '../calibration/rm_vlc_leftfront'

# Video Encoding profiles
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 1*1024*1024

# Marker properties
radius = 5
color = (255, 255, 0)
thickness = 3

#------------------------------------------------------------------------------

calibration = hl2ss.download_calibration_rm_vlc(host, port)
model_vlc = hl2ss_utilities.rm_vlc_load_pinhole_model(model_path)

enable     = True
last_frame = None
last_pose  = None
last_left  = None
last_right = None

def recv_pv():
    global enable
    global last_frame
    global last_pose

    client = hl2ss_utilities.rx_decoded_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_1, profile, bitrate)
    client.open()
    while (enable):
        data = client.get_next_packet()
        last_pose = data.pose
        last_frame = data.payload
    client.close()

def recv_si():
    global enable
    global last_left
    global last_right

    client = hl2ss.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)
    client.open()
    while (enable):
        data = client.get_next_packet()
        si = hl2ss.unpack_si(data.payload)
        last_left  = si.get_hand_left()  if (si.is_valid_hand_left())  else None
        last_right = si.get_hand_right() if (si.is_valid_hand_right()) else None
    client.close()

thread_pv = threading.Thread(target=recv_pv)
thread_si = threading.Thread(target=recv_si)

thread_pv.start()
thread_si.start()

def render_hand(image, hand, P, radius, color, thickness):
    _, _, positions, _, _ = hl2ss_utilities.si_unpack_hand(hand)
    ipoints, _ = hl2ss_utilities.project_to_image(hl2ss_utilities.to_homogeneous(positions), P)
    for x, y in ipoints:
        cv2.circle(image, (int(x), (int(y))), radius, color, thickness)

try:
    while (True):
        if (last_frame is None):
            continue

        image = hl2ss_utilities.rm_vlc_undistort(last_frame, model_vlc.map)
        next_pose  = last_pose
        next_left  = last_left
        next_right = last_right

        P = hl2ss_utilities.projection(model_vlc.intrinsics, hl2ss_utilities.rm_world_to_camera(calibration.extrinsics, next_pose))

        if (next_left is not None):
            render_hand(image, next_left, P, radius, color, thickness)

        if (next_right is not None):
            render_hand(image, next_right, P, radius, color, thickness)

        cv2.imshow('Video', image)
        cv2.waitKey(1)
except:
    pass

enable = False
thread_pv.join()
thread_si.join()
