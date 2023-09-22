#------------------------------------------------------------------------------
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
<<<<<<< HEAD:viewer/client_rm_vlc.py
import configparser
=======
import hl2ss_lnm
>>>>>>> 5d92301451f23c976ebcf6f65a35728896a2bb09:viewer/client_stream_rm_vlc.py

# Settings --------------------------------------------------------------------
config = configparser.ConfigParser()
config.read('config.ini')
# HoloLens address
host = config['DEFAULT']['ip']

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

# Video encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

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

client = hl2ss_lnm.rx_rm_vlc(host, port, mode=mode, divisor=divisor, profile=profile)
client.open()

while (enable):
    data = client.get_next_packet()
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    cv2.imshow('Video', data.payload)
    cv2.waitKey(1)

client.close()
listener.join()
