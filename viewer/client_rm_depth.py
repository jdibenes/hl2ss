#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens depth camera and plays
# it. Only long throw mode is supported. The resolution is 320x288 5 FPS. The
# stream supports three operating modes: 0) video, 1) video + rig pose,
# 2) query calibration (single transfer).
#------------------------------------------------------------------------------

import hl2ss 
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
    data = hl2ss.download_calibration_rm_depth(host, port)
    print('Calibration data')
    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.scale)
    quit()

pose_printer = hl2ss.pose_printer(10)
fps_counter = hl2ss.framerate_counter(10)
glitch_detector = hl2ss.continuity_analyzer(hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS / hl2ss.Resolution_RM_DEPTH_LONGTHROW.FPS)
client = hl2ss.connect_client_rm_depth(host, port, 4096, mode)

try:
    while True:
        data      = client.get_next_packet()
        timestamp = data.timestamp
        images    = hl2ss.unpack_rm_depth(data.payload)
        depth     = images.depth
        ab        = images.ab

        glitch_detector.push(timestamp)
        fps_counter.push()
        pose_printer.push(timestamp, data.pose)
        
        cv2.imshow('depth', depth*brightness)
        cv2.imshow('ab', ab*brightness)
        cv2.waitKey(1)
except:
    pass

client.close()
