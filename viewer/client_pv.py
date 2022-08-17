#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens front RGB camera and
# plays it. The camera support various resolutions and framerates. See
# etc/hl2_capture_formats.txt for a list of supported formats. The default
# configuration is 1080p 30 FPS. The stream supports three operating modes:
# 0) video, 1) video + camera pose, 2) query calibration (single transfer).
#------------------------------------------------------------------------------

import hl2ss
import hl2ss_utilities
import cv2
import av

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

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

# Print period
# Print camera pose every period frames
period = 60

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_pv(host, port, width, height, framerate, profile, bitrate)
    print('Calibration')
    print(data.focal_length)
    print(data.principal_point)
    print(data.radial_distortion)
    print(data.tangential_distortion)
    print(data.projection)
    quit()

codec = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')
pose_printer = hl2ss_utilities.pose_printer(period)
client = hl2ss.rx_pv(host, port, hl2ss.ChunkSize.PERSONAL_VIDEO, mode, width, height, framerate, profile, bitrate)
client.open()

try:
    while True:
        data = client.get_next_packet()
        timestamp = data.timestamp
        pose_printer.push(timestamp, data.pose)
        for packet in codec.parse(data.payload):
            for frame in codec.decode(packet):
                image = frame.to_ndarray(format='bgr24')
                cv2.imshow('Video', image)
                cv2.waitKey(1)
except:
    pass

client.close()
