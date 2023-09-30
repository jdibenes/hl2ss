#------------------------------------------------------------------------------
# This script receives video from the HoloLens depth camera in long throw mode
# and plays it. The resolution is 320x288 @ 5 FPS. The stream supports three
# operating modes: 0) video, 1) video + rig pose, 2) query calibration (single 
# transfer). Depth and AB data are scaled for visibility. The ahat and long 
# throw streams cannot be used simultaneously.
# Press esc to stop. 
#------------------------------------------------------------------------------

from pynput import keyboard
import os
import calendar
import time
import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
#<<<<<<< HEAD:viewer/client_rm_depth_longthrow.py
import configparser
#=======
import hl2ss_lnm
#>>>>>>> 5d92301451f23c976ebcf6f65a35728896a2bb09:viewer/client_stream_rm_depth_longthrow.py

save = True
save_path = './capture_frames/longthrow/'
# Settings --------------------------------------------------------------------
config = configparser.ConfigParser()
config.read('config.ini')
# HoloLens address
host = config['DEFAULT']['ip']

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (singlse transfer)
mode = hl2ss.StreamMode.MODE_1

# Framerate denominator (must be > 0)
# Effective framerate is framerate / divisor
divisor = 1

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

client = hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, mode=mode, divisor=divisor)
client.open()

while (enable):
    data = client.get_next_packet()

    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    
    cv2.imshow('Depth', data.payload.depth / np.max(data.payload.depth)) # Scaled for visibility
    cv2.imshow('AB', data.payload.ab / np.max(data.payload.ab)) # Scaled for visibility
    if save:
        #calendar.timegm(gmt)
        gmt = time.gmtime()
        cv2.imwrite(os.path.join(save_path, str(calendar.timegm(gmt)) + '.png'), data.payload.depth)

    cv2.waitKey(1)
    #cv2.imwrite(filename, img)
client.close()
listener.join()
