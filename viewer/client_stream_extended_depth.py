#------------------------------------------------------------------------------
# Prototype for receiving depth frames from an Intel RealSense depth camera
# connected to the HoloLens.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Camera selection
group_index = 0
source_index = 0
profile_index = 0
media_index = 15

# Encoding
profile_z = hl2ss.DepthProfile.ZDEPTH

#------------------------------------------------------------------------------

hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH, global_opacity=group_index, output_width=source_index, output_height=profile_index)

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

client = hl2ss_lnm.rx_extended_depth(host, hl2ss.StreamPort.EXTENDED_DEPTH, profile_z=profile_z, media_index=media_index)
client.open()

max_depth = 8192 # Depends on your RGBD camera
max_uint8 = 255

while (enable):
    data = client.get_next_packet()

    print(f'Frame captured at {data.timestamp} with resolution {data.payload.width}x{data.payload.height}')

    depth = data.payload.depth

    cv2.imshow('Depth', cv2.applyColorMap(((depth / max_depth) * max_uint8).astype(np.uint8), cv2.COLORMAP_JET))
    cv2.waitKey(1)

client.close()
listener.join()

hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH)
