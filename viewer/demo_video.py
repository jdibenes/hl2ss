#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens cameras and plays it.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import multiprocessing as mp
import cv2
import hl2ss
import hl2ss_mp

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Ports
ports = [
    hl2ss.StreamPort.PERSONAL_VIDEO,
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    #hl2ss.StreamPort.RM_DEPTH_AHAT,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW
    ]

# RM VLC parameters
vlc_mode    = hl2ss.StreamMode.MODE_1
vlc_profile = hl2ss.VideoProfile.H264_BASE
vlc_bitrate = 1*1024*1024

# RM Depth AHAT parameters
ahat_mode = hl2ss.StreamMode.MODE_1
ahat_profile = hl2ss.VideoProfile.H264_BASE
ahat_bitrate = 8*1024*1024

# RM Depth Long Throw parameters
lt_mode = hl2ss.StreamMode.MODE_1
lt_filter = hl2ss.PngFilterMode.Paeth

# PV parameters
pv_mode = hl2ss.StreamMode.MODE_1
pv_width = 1280
pv_height = 720
pv_framerate = 30
pv_profile = hl2ss.VideoProfile.H265_MAIN
pv_bitrate = 5*1024*1024
pv_format = 'bgr24'

# Maximum number of frames in buffer
buffer_elements = 60

#------------------------------------------------------------------------------

if __name__ == '__main__':
    if ((hl2ss.StreamPort.PERSONAL_VIDEO in ports) and (hl2ss.StreamPort.RM_DEPTH_AHAT in ports)):
        print('Error: Simultaneous PV and RM Depth AHAT streaming is not supported. See known issues at https://github.com/jdibenes/hl2ss.')
        quit()

    if ((hl2ss.StreamPort.RM_DEPTH_LONGTHROW in ports) and (hl2ss.StreamPort.RM_DEPTH_AHAT in ports)):
        print('Error: Simultaneous RM Depth Long Throw and RM Depth AHAT streaming is not supported. See known issues at https://github.com/jdibenes/hl2ss.')
        quit()

    client_rc = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)

    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
        client_rc.wait_for_pv_subsystem(True)

    producer = hl2ss_mp.producer()
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_depth_ahat(True, host, hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss.ChunkSize.RM_DEPTH_AHAT, ahat_mode, ahat_profile, ahat_bitrate)
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, lt_mode, lt_filter)
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, pv_mode, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate, pv_format)

    for port in ports:
        producer.initialize(port, buffer_elements)
        producer.start(port)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sinks = {}

    for port in ports:
        sinks[port] = consumer.create_sink(producer, port, manager, None)
        sinks[port].get_attach_response()

    def display_basic(port, payload):
        cv2.imshow(hl2ss.get_port_name(port), payload)

    def display_depth(port, payload):
        cv2.imshow(hl2ss.get_port_name(port) + '-depth', payload.depth * 8) # Scaled for visibility
        cv2.imshow(hl2ss.get_port_name(port) + '-ab', payload.ab / np.max(payload.ab)) # Normalized for visibility

    DISPLAY_MAP = {
        hl2ss.StreamPort.RM_VLC_LEFTFRONT   : display_basic,
        hl2ss.StreamPort.RM_VLC_LEFTLEFT    : display_basic,
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT  : display_basic,
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT  : display_basic,
        hl2ss.StreamPort.RM_DEPTH_AHAT      : display_depth,
        hl2ss.StreamPort.RM_DEPTH_LONGTHROW : display_depth,
        hl2ss.StreamPort.PERSONAL_VIDEO     : display_basic
    }

    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while (enable):
        for port in ports:
            data = sinks[port].get_most_recent_frame()
            if (data is not None):
                DISPLAY_MAP[port](port, data.payload)
        cv2.waitKey(1)

    for port in ports:
        sinks[port].detach()

    for port in ports:
        producer.stop(port)

    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
        client_rc.wait_for_pv_subsystem(False)

    listener.join()
