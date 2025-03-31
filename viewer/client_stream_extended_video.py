#------------------------------------------------------------------------------
# This script describes how to receive video from an external USB-C camera
# connected to the HoloLens.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import json
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Operating mode
# 0: video
# 1: video + rignode pose
# 2: query devices (single transfer)
# Mode 2 as default since the user has to find their camera in the device list
mode = hl2ss.StreamMode.MODE_2

# Camera selection
# 1. Connect your external USB-C camera to the HoloLens 
# 2. Call hl2ss_lnm.download_devicelist_extended_video to obtain a dictionary
#    (see media_groups below) describing all video capture devices
# 3. Find the values of group_index, source_index, and profile_index for your
#    camera and desired resolution in the dictionary, which has this form:
#      media_groups[group_index]['SourceInfos'][source_index]
#      media_groups[group_index]['VideoProfiles'][profile_index]
#    If 'VideoProfiles' is empty, then set profile_index = 0
#    The built-in PV camera is included in the dictionary and can be selected
#    If no external cameras are connected, the values for the PV camera should
#    be group_index = 0, source_index = 2, and profile_index = 4
#    Only configurations with Subtype(s) NV12, YUY2, IYUV, I420, or YV12 and 
#    MediaStreamType = 1, MediaSourceKind = 1 are supported
group_index = 0
source_index = 2
profile_index = 4

# Enable Shared Capture
# If another program is already using your camera, you can still stream it by
# enabling shared mode, however you cannot change the resolution and framerate
# As an example, you can run the PV camera client script with Mixed Reality
# Capture enabled to display holograms in the video, then you can run this
# script with the PV camera selected and shared = True to obtain another PV
# stream without holograms (the extended video interface does not support MRC)
shared = False

# Camera parameters
# Must exist in media_groups[group_index]['SourceInfos'][source_index] and
# media_groups[group_index]['VideoProfiles'][profile_index] if any
# Ignored in shared mode
width     = 1280
height    = 720
framerate = 30

# Video encoding profile and bitrate (None = default)
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = None

# Decoded format
# Options include:
# 'bgr24'
# 'rgb24'
# 'bgra'
# 'rgba'
# 'gray8'
# If profile is hl2ss.VideoProfile.RAW then these conversions are only 
# supported if the video Subtype is NV12
# For other Subtypes, the user has to cast the image to the appropriate shape
# and type before converting to the desired format
# Example: 
# For RAW video profile and YUY2 Subtype, convert YUY2 to BGR using OpenCV
#     decoded_format = 'any'
#     ...
#     data.payload.image = data.payload.image.reshape((data.payload.resolution[1], data.payload.resolution[0], 2))
#     data.payload.image = cv2.cvtColor(data.payload.image, cv2.COLOR_YUV2BGR_YUY2)
decoded_format = 'bgr24'

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    media_groups = json.loads(hl2ss_lnm.download_devicelist_extended_video(host, hl2ss.StreamPort.EXTENDED_VIDEO))
    print(json.dumps(media_groups, indent=2))
    quit()


hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO, shared=shared, global_opacity=group_index, output_width=source_index, output_height=profile_index)

listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
listener.open()

client = hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO, mode=mode, width=width, height=height, framerate=framerate, profile=profile, bitrate=bitrate, decoded_format=decoded_format)
client.open()

while (not listener.pressed()):
    data = client.get_next_packet()

    print(f'Frame captured at {data.timestamp} with resolution {data.payload.resolution}')
    print(f'Pose')
    print(data.pose)

    cv2.imshow('Video', data.payload.image)
    cv2.waitKey(1)

client.close()
listener.close()

hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO)
