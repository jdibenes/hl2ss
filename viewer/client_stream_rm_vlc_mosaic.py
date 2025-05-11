#------------------------------------------------------------------------------
# This script receives video from one of the HoloLens sideview grayscale
# cameras and plays it. The camera resolution is 640x480 @ 30 FPS. The stream 
# supports three operating modes: 0) video, 1) video + rig pose, 2) query 
# calibration (single transfer).
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
# Options:
# hl2ss.StreamPort.RM_VLC_LEFTFRONT
# hl2ss.StreamPort.RM_VLC_LEFTLEFT
# hl2ss.StreamPort.RM_VLC_RIGHTFRONT
# hl2ss.StreamPort.RM_VLC_RIGHTRIGHT
port = hl2ss.StreamPort.RM_VLC_MOSAIC

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Video encoding profile and bitrate (None = default)
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = None

#------------------------------------------------------------------------------

listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
listener.open()

client = hl2ss.rx_decoded_rm_vmu(host, port, hl2ss_lnm.create_sockopt(), 4096, mode | 0xF0, 1, profile, hl2ss.H26xLevel.DEFAULT, (640*480*30*12*6)//100, hl2ss_lnm.get_video_codec_default_options(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, 1, profile))
client.open()

while (not listener.pressed()):
    data = client.get_next_packet()

    print(f'Frame captured at {data.timestamp}')
    #print(f'Sensor Ticks: {data.payload.sensor_ticks}')
    #print(f'Exposure: {data.payload.exposure}')
    #print(f'Gain: {data.payload.gain}')
    print(f'Pose')
    print(data.pose)

    cv2.imshow('Video 0', data.payload.image[0])
    cv2.imshow('Video 1', data.payload.image[1])
    cv2.imshow('Video 2', data.payload.image[2])
    cv2.imshow('Video 3', data.payload.image[3])
    cv2.waitKey(1)

client.close()
listener.close()
