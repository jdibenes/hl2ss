#------------------------------------------------------------------------------
# This script receives encoded video from one of the HoloLens sideview
# grayscale cameras and plays it. The camera resolution is 640x480 @ 30 FPS.
# The stream supports three operating modes: 0) video, 1) video + rig pose,
# 2) query calibration (single transfer). Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss_imshow
import hl2ss

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

# Video encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 1*1024*1024

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_vlc(host, port)
    print('Calibration data')
    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.undistort_map.shape)
    print(data.intrinsics)
    quit()

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

client = hl2ss.rx_decoded_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, mode, profile, bitrate)
client.open()

while (enable):
    data = client.get_next_packet()
    print('Pose at time {ts}'.format(ts=data.timestamp))
    print(data.pose)
    cv2.imshow('Video', data.payload)
    cv2.waitKey(1)

client.close()
listener.join()
