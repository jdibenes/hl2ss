#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens depth camera and plays
# it. Only long throw mode is supported. The resolution is 320x288 5 FPS. The
# stream supports three operating modes: 0) video, 1) video + rig pose,
# 2) query calibration (single transfer). Press esc to stop. Depth and AB
# data are scaled for visibility.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import cv2

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
port = hl2ss.StreamPort.RM_DEPTH_AHAT

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_0

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_depth_ahat(host, port)
    print('Calibration data')
    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.scale)
    print(data.alias)
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

client = hl2ss.rx_decoded_rm_depth_ahat(host, port, hl2ss.ChunkSize.RM_DEPTH_AHAT, mode, hl2ss.VideoProfile.H264_BASE, 8*1024*1024)
client.open()

prev_ts = None
count = 0

while (enable):
    data = client.get_next_packet()
    cv2.imshow('depth', data.payload.depth)
    cv2.imshow('ab', data.payload.ab)
    cv2.waitKey(1)

    if (prev_ts is not None):
        print((10*1000*1000) / (data.timestamp - prev_ts))
    prev_ts = data.timestamp
    print(data.pose)

client.close()
listener.join()
