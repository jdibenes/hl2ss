#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens front RGB camera and
# plays it. The camera support various resolutions and framerates. See
# etc/hl2_capture_formats.txt for a list of supported formats. The default
# configuration is 1080p 30 FPS. The stream supports three operating modes:
# 0) video, 1) video + camera pose, 2) query calibration (single transfer).
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import cv2

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
port = hl2ss.StreamPort.PERSONAL_VIDEO

# Operating mode
# 0: video
# 1: video + camera pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Camera parameters
width     = 1920
height    = 1080
framerate = 30

# Video encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 5*1024*1024

#------------------------------------------------------------------------------

client = hl2ss.rx_decoded_pv(host, port, hl2ss.ChunkSize.PERSONAL_VIDEO, mode, width, height, framerate, profile, bitrate, 'bgr24')
client.start_video_subsystem()

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_pv(host, port, width, height, framerate)
    print('Calibration')
    print(data.focal_length)
    print(data.principal_point)
    print(data.radial_distortion)
    print(data.tangential_distortion)
    print(data.projection)
    print(data.intrinsics)
else:
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    client.open()

    while (enable):
        data = client.get_next_packet()
        print('Pose at time {ts}'.format(ts=data.timestamp))
        print(data.pose)
        cv2.imshow('Video', data.payload)
        cv2.waitKey(1)

    client.close()
    listener.join()

client.stop_video_subsystem()
