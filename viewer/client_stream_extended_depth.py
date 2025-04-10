#------------------------------------------------------------------------------
# Prototype for receiving depth frames from an Intel RealSense depth camera
# connected to the HoloLens.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Operating mode
# 0: video
# 1: video + rignode pose
mode = hl2ss.StreamMode.MODE_1

# Camera selection
# Use the Extended Video device list (see client_stream_extended_video.py) to
# find the group index, source_index, and profile_index of your depth camera
# For RealSense D435i, use source_index = 0, profile_index = 0, and select
# media_index from
# https://github.com/jdibenes/hl2ss/blob/main/etc/ez_realsense_d435i.txt
group_index   = 0
source_index  = 0
profile_index = 0
media_index   = 13 # 13: 640x360 @ 60 FPS 

# Depth Encoding
profile_z = hl2ss.DepthProfile.ZDEPTH

# Maximum depth (depends on your RGBD camera)
max_depth = 8192 

#------------------------------------------------------------------------------

hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH, global_opacity=group_index, output_width=source_index, output_height=profile_index)

listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
listener.open()

client = hl2ss_lnm.rx_extended_depth(host, hl2ss.StreamPort.EXTENDED_DEPTH, mode=mode, profile_z=profile_z, media_index=media_index)
client.open()

while (not listener.pressed()):
    data = client.get_next_packet()

    print(f'Frame captured at {data.timestamp} with resolution {data.payload.resolution}')
    print(f'Pose')
    print(data.pose)

    depth = data.payload.depth

    cv2.imshow('Depth', hl2ss_3dcv.rm_depth_colormap(depth, max_depth))
    cv2.waitKey(1)

client.close()
listener.close()

hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH)
