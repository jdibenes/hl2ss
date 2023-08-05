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
import hl2ss_imshow
import hl2ss
import hl2ss_utilities
import struct
import pyzdepth

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

# Framerate denominator (must be > 0)
# Effective framerate is framerate / divisor
divisor = 1 

# Video encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

# Group of pictures (GOP) size
gop_size = hl2ss.get_video_codec_default_gop_size(hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor)

# Encoded stream average bits per second
# Must be > 0
bitrate = 2*1024*1024

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_depth_ahat(host, port)
    print('Calibration data')
    print('Image point to unit plane')
    print(data.uv2xy)
    print('Extrinsics')
    print(data.extrinsics)
    print(f'Scale: {data.scale}')
    print(f'Alias: {data.alias}')
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

qpc = hl2ss_utilities.framerate_counter()

client = hl2ss.rx_rm_depth_ahat(host, port, hl2ss.ChunkSize.RM_DEPTH_AHAT, mode, divisor, profile, gop_size, bitrate)
client.open()

qpc.reset()

decompressor = pyzdepth.DepthCompressor()

while (enable):
    data = client.get_next_packet()
    ab_size, z_size = struct.unpack_from('<II', data.payload, 0)
    z_data = bytes(data.payload[(8 + ab_size):])    
    result, width, height, decompressed = decompressor.Decompress(z_data)
    depth = np.frombuffer(decompressed, dtype=np.uint16).reshape((height, width))
    cv2.imshow('Depth', depth / np.max(depth)) # Scaled for visibility
    cv2.waitKey(1)
    #print(f'{struct.unpack("<II", data.payload[0:8])}')
    #print(f'Pose at time {data.timestamp}')
    #print(data.pose)
    #cv2.imshow('Depth', data.payload.depth / np.max(data.payload.depth)) # Scaled for visibility
    #cv2.imshow('AB', data.payload.ab / np.max(data.payload.ab)) 
    #cv2.waitKey(1)
    qpc.increment()
    if (qpc.delta() > 3):
        print(f'FPS: {qpc.get()}') # got 45 FPS!
        qpc.reset()

client.close()
listener.join()
