#------------------------------------------------------------------------------
# C++ Multithreaded Client Python Extension Test.
# Requires building the hl2ss_ulm_stream extension.
#
# This script receives video from the HoloLens depth camera in long throw mode
# and plays it. The resolution is 320x288 @ 5 FPS. The stream supports three
# operating modes: 0) video, 1) video + rig pose, 2) query calibration (single 
# transfer). Depth and AB data are scaled for visibility. The ahat and long 
# throw streams cannot be used simultaneously.
# Press esc to stop. 
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mt

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Framerate denominator (must be > 0)
# Effective framerate is framerate / divisor
divisor = 1

# Buffer size (seconds)
buffer_size = 5

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss_lnm.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    print('Calibration data')
    print('Image point to unit plane')
    print(data.uv2xy)
    print('Extrinsics')
    print(data.extrinsics)
    print(f'Scale: {data.scale}')
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

configuration = hl2ss_mt.create_configuration(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
configuration['mode'] = mode
configuration['divisor'] = divisor

client = hl2ss_mt.rx_decoded_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, buffer_size * hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS, configuration)
client.open()

max_depth = 7500
max_uint8 = 255

cv2.namedWindow('Depth')
cv2.namedWindow('AB')

while (enable):
    cv2.waitKey(1)

    data = client.get_by_index(-1)
    if (data.status != hl2ss_mt.Status.OK):
        continue

    print(f'Frame captured at {data.timestamp}')
    print(f'Sensor Ticks: {data.payload.sensor_ticks}')
    print(f'Pose')
    print(data.pose)

    depth = data.payload.depth
    ab = data.payload.ab
    
    cv2.imshow('Depth', cv2.applyColorMap(((depth / max_depth) * max_uint8).astype(np.uint8), cv2.COLORMAP_JET)) # Scaled for visibility
    cv2.imshow('AB', np.sqrt(ab).astype(np.uint8)) # Scaled for visibility

client.close()
listener.join()
