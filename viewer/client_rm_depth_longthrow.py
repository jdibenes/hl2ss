#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens depth camera in long 
# throw mode and plays it. The resolution is 320x288 @ 5 FPS. The stream
# supports three operating modes: 0) video, 1) video + rig pose, 2) query
# calibration (single transfer). Press esc to stop. Depth and AB data are 
# scaled for visibility. Note that the ahat and long throw streams cannot be
# used simultaneously.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import cv2
import hl2ss_imshow
import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# PNG filter
png_filter = hl2ss.PngFilterMode.Paeth

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_depth_longthrow(host, port)
    print('Calibration data')
    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.scale)
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

client = hl2ss.rx_decoded_rm_depth_longthrow(host, port, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode, png_filter)
client.open()

while (enable):
    data = client.get_next_packet()
    print('Pose at time {ts}'.format(ts=data.timestamp))
    print(data.pose)
    cv2.imshow('Depth', data.payload.depth / np.max(data.payload.depth)) # Normalized for visibility
    cv2.imshow('AB', data.payload.ab / np.max(data.payload.ab)) # Normalized for visibility
    cv2.waitKey(1)

client.close()
listener.join()
