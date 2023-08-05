#------------------------------------------------------------------------------
# This script receives video from the HoloLens depth camera in ahat mode and 
# plays it. The resolution is 512x512 @ 45 FPS. The stream supports three 
# operating modes: 0) video, 1) video + rig pose, 2) query calibration (single 
# transfer). Depth and AB data are scaled for visibility. The ahat and long 
# throw streams cannot be used simultaneously.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import cv2
import hl2ss

import logging

log = logging.getLogger(__name__)

# Settings --------------------------------------------------------------------
DEFAULT_KEY = "tcn/loc/hl2/*"
# most simple zenoh config for now
conf = {"mode": "peer", "queries_default_timeout": 10000}


# Video encoding profile
profile = hl2ss.VideoProfile.H264_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 8*1024*1024

#------------------------------------------------------------------------------

# data = hl2ss.download_calibration_rm_depth_ahat(host, port)
# print('Calibration data')
# print('Image point to unit plane')
# print(data.uv2xy)
# print('Extrinsics')
# print(data.extrinsics)
# print(f'Scale: {data.scale}')
# print(f'Alias: {data.alias}')
# print('Undistort map')
# print(data.undistort_map)
# print('Intrinsics (undistorted only)')
# print(data.intrinsics)
# quit()

enable = True

logging.basicConfig(level=logging.DEBUG)

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

client = hl2ss.rx_decoded_rm_depth_ahat(conf, DEFAULT_KEY, profile)
client.open()

while (enable):
    data = client.get_next_packet()
    if data is None:
        print("no data")
        cv2.waitKey(1)
        continue
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    if data.payload.depth:
        cv2.imshow('Depth', data.payload.depth / np.max(data.payload.depth)) # Scaled for visibility
    if data.payload.ab:
        cv2.imshow('AB', data.payload.ab / np.max(data.payload.ab)) # Scaled for visibility
    cv2.waitKey(1)

client.close()
listener.join()
