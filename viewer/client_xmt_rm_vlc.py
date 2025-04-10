#------------------------------------------------------------------------------
# C++ Multithreaded Client Python Extension Test.
# Requires building the hl2ss_ulm_stream extension.
#
# This script receives video from one of the HoloLens sideview grayscale
# cameras and plays it. The camera resolution is 640x480 @ 30 FPS. The stream 
# supports three operating modes: 0) video, 1) video + rig pose, 2) query 
# calibration (single transfer).
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mt

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
# Options:
# hl2ss.StreamPort.RM_VLC_LEFTFRONT
# hl2ss.StreamPort.RM_VLC_LEFTLEFT
# hl2ss.StreamPort.RM_VLC_RIGHTFRONT
# hl2ss.StreamPort.RM_VLC_RIGHTRIGHT
port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Framerate denominator (must be > 0)
# Effective framerate is framerate / divisor
divisor = 1

# Video encoding profile and bitrate (None = default)
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = None

# Buffer size (seconds)
buffer_size = 5

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss_lnm.download_calibration_rm_vlc(host, port)
    print('Calibration data')
    print('Image point to unit plane')
    print(data.uv2xy)
    print('Extrinsics')
    print(data.extrinsics)
    print('Undistort map')
    print(data.undistort_map)
    print('Intrinsics (undistorted only)')
    print(data.intrinsics)
    quit()

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

configuration = hl2ss_mt.create_configuration(port)
configuration['mode'] = mode
configuration['divisor'] = divisor
configuration['profile'] = profile
configuration['bitrate'] = bitrate

client = hl2ss_mt.rx_decoded_rm_vlc(host, port, buffer_size * hl2ss.Parameters_RM_VLC.FPS, configuration)
client.open()

cv2.namedWindow('Video')

while (enable):
    cv2.waitKey(1)

    data = client.get_by_index(-1)
    if (data.status != hl2ss_mt.Status.OK):
        continue

    print(f'Frame captured at {data.timestamp}')
    print(f'Sensor Ticks: {data.payload.sensor_ticks}')
    print(f'Exposure: {data.payload.exposure}')
    print(f'Gain: {data.payload.gain}')
    print(f'Pose')
    print(data.pose)

    cv2.imshow('Video', data.payload.image)

client.close()
listener.join()
