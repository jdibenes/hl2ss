#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens depth camera and plays
# it. Only long throw mode is supported. The resolution is 320x288 5 FPS. The
# stream supports three operating modes: 0) video, 1) video + rig pose,
# 2) query calibration (single transfer).
#------------------------------------------------------------------------------

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

# Print period
# Print rig pose every period frames
period = 10

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_depth_longthrow(host, port)
    print('Calibration data')
    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.scale)
    quit()

pose_printer = hl2ss_utilities.pose_printer(period)
client = hl2ss.rx_rm_depth(host, port, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode)
client.open()

try:
    while True:
        data      = client.get_next_packet()
        timestamp = data.timestamp
        images    = hl2ss.unpack_rm_depth(data.payload)
        depth     = images.depth
        ab        = images.ab

        pose_printer.push(timestamp, data.pose)
        
        cv2.imshow('depth', depth*brightness)
        cv2.imshow('ab', ab*brightness)
        cv2.waitKey(1)
except:
    pass

client.close()
