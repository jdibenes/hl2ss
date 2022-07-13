#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens front RGB camera and
# plays it. The camera support various resolutions and framerates. See
# etc/hl2_capture_formats.txt for a list of supported formats. The default
# configuration is 1080p 30 FPS. The stream supports three operating modes:
# 0) video, 1) video + camera pose, 2) query intrinsics (single transfer).

import hl2ss
import cv2
import av

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port number
# Do not edit
port = hl2ss.StreamPort.PERSONAL_VIDEO

# Operating mode
# 0: video
# 1: video + camera pose
# 2: query intrinsics
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

# Maximum number of bytes to read from the socket buffer per step
# Use an appropriate power-of-two value
chunk_size = 8192

# FPS period
# Display fps every 'fps_period' frames
fps_period = 60

# Pose period
# Display pose every 'pose_period' frames if streaming in mode 1
# Set to 1 to display pose for each frame
pose_period = 60

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.get_mode2_pv(host, port, width, height, framerate, profile, bitrate)

    print('Camera intrinsics')

    print('Focal length')
    print(data.focal_length)

    print('Principal point')
    print(data.principal_point)

    print('Radial distortion')
    print(data.radial_distortion)

    print('Tangential distortion')
    print(data.tangential_distortion)

    print('Projection')
    print(data.projection)

    quit()

# Modes 0 and 1
client = hl2ss.gatherer()
fps_counter = hl2ss.framerate_counter()
pose_counter = hl2ss.counter()
sequence_checker = hl2ss.continuity_analyzer(hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS / framerate)
codec = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')

client.open(host, port, chunk_size, mode)
client.configure(hl2ss.create_configuration_for_video(mode, width, height, framerate, profile, bitrate))

while True:
    data = client.get_next_packet()

    timestamp = data.timestamp
    packets = codec.parse(data.payload)
    for packet in packets:
        for frame in codec.decode(packet):
            image = frame.to_ndarray(format='bgr24')

            # Check for frame drops
            if (sequence_checker.check(timestamp)):
                print('FRAME DROP DETECTED')

            # Display FPS
            if (fps_counter.increment() >= fps_period):
                print("FPS {fps}".format(fps=fps_counter.pop()))

            # Display Pose
            if ((mode == hl2ss.StreamMode.MODE_1) and (pose_counter.increment() >= pose_period)):
                pose_counter.reset()
                print('Pose at {timestamp}'.format(timestamp=timestamp))
                print(data.pose)

            cv2.imshow('Video', image)
            cv2.waitKey(1)
                