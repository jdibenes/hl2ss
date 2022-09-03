#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens depth camera and plays
# it. Only long throw mode is supported. The resolution is 320x288 5 FPS. The
# stream supports three operating modes: 0) video, 1) video + rig pose,
# 2) query calibration (single transfer). Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import hl2ss_utilities
import cv2

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Scaling factor for visibility
brightness = 8

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_depth_longthrow(host, port)
    print('Calibration data')
    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.scale)
    quit()

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

client = hl2ss_utilities.rx_decoded_rm_depth(host, port, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode)
client.open()

while (enable):
    data = client.get_next_packet()
    print('Pose at time {ts}'.format(ts=data.timestamp))
    print(data.pose)
    cv2.imshow('depth', data.payload.depth*brightness)
    cv2.imshow('ab', data.payload.ab*brightness)
    cv2.waitKey(1)

client.close()
listener.join()
