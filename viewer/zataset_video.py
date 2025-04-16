# Dataset capture for vo/slam experiments -------------------------------------

from datetime import datetime
from pynput import keyboard

import os
import time
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_mx
import hl2ss_mp
import hl2ss_ds
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Output directory
path = './data'

# Socket timeout
timeout = 30

# Encoding profile for video streams
video_profile = hl2ss.VideoProfile.H265_MAIN

# PV parameters
pv_width = 760
pv_height = 428
pv_framerate = 30

# User data
user_data = 'slam dataset'.encode()

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Wait for start signal ---------------------------------------------------
    ports = [
        hl2ss.StreamPort.RM_VLC_LEFTFRONT,
        hl2ss.StreamPort.RM_VLC_LEFTLEFT,
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
        hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
        hl2ss.StreamPort.PERSONAL_VIDEO,
    ]

    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()

    print('Press space to start recording...')
    while (not listener.pressed()):
        time.sleep(1/60)
    print('Preparing...')

    listener.close()

    # Generate output filenames -----------------------------------------------
    now    = datetime.now()
    folder = os.path.join(path, f'HL2-{now.year:04d}-{now.month:02d}-{now.day:02d}-{now.hour:02d}-{now.minute:02d}-{now.second:02d}')

    # Create output folder
    # Fail if folder exists to avoid overwriting previous data, if any
    os.makedirs(folder)

    filenames = {port : os.path.join(folder, f'{hl2ss.get_port_name(port)}.bin') for port in ports}

    # Port configuration ------------------------------------------------------
    sockopt = hl2ss_lnm.create_sockopt(settimeout=timeout)

    # Start subsystem ---------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, sockopt)

    # Configure system --------------------------------------------------------
    client_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION, sockopt)
    client_rc.open()
    client_rc.pv_wait_for_subsystem(True)
    client_rc.pv_set_focus(hl2ss.PV_FocusMode.Manual, hl2ss.PV_AutoFocusRange.Normal, hl2ss.PV_ManualFocusDistance.Infinity, 3000, hl2ss.PV_DriverFallback.Disable)
    client_rc.pv_set_exposure(hl2ss.PV_ExposureMode.Manual, 16666)
    client_rc.pv_set_exposure_priority_video(hl2ss.PV_ExposurePriorityVideo.Disabled)
    client_rc.pv_set_white_balance_preset(hl2ss.PV_ColorTemperaturePreset.Flash)
    client_rc.pv_set_video_temporal_denoising(hl2ss.PV_VideoTemporalDenoisingMode.Off)
    client_rc.pv_set_backlight_compensation(hl2ss.PV_BacklightCompensationState.Disable)
    client_rc.ee_set_interface_priority(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.EE_InterfacePriority.HIGHEST)
    client_rc.ee_set_interface_priority(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss.EE_InterfacePriority.HIGHEST)
    client_rc.ee_set_interface_priority(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss.EE_InterfacePriority.HIGHEST)
    client_rc.ee_set_interface_priority(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss.EE_InterfacePriority.HIGHEST)
    client_rc.ee_set_interface_priority(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.EE_InterfacePriority.ABOVE_NORMAL)
    client_rc.ee_set_encoder_buffering(True)
    client_rc.close()
   
    # Start receivers ---------------------------------------------------------
    
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, sockopt, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, sockopt, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, sockopt, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, sockopt, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, sockopt, decoded=False))
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, sockopt, width=pv_width, height=pv_height, framerate=pv_framerate, profile=video_profile, decoded_format=None))

    consumer = hl2ss_mp.consumer()
    sinks = {}

    for port in ports:
        producer.initialize(port)
        producer.start(port)
        sinks[port] = consumer.get_default_sink(producer, port)
        sinks[port].get_attach_response()
        while (sinks[port].get_buffered_frame(-1)[0] != hl2ss_mx.Status.OK):
            pass
        print(f'Started stream {hl2ss.get_port_name(port)}')

    writers = {port : hl2ss_ds.wr(filenames[port], producer, port, user_data) for port in ports}

    for port in ports:
        writers[port].open()
        print(f'Started writer {hl2ss.get_port_name(port)}')

    # Wait for stop signal ----------------------------------------------------
    print('Recording started.')

    decoder_pv = hl2ss.decode_pv(video_profile)
    sink_pv = sinks[hl2ss.StreamPort.PERSONAL_VIDEO]
    fs_pv = -1
    window_name_pv = hl2ss.get_port_name(hl2ss.StreamPort.PERSONAL_VIDEO)

    cv2.namedWindow(window_name_pv)
    
    print('Press esc to stop recording...')

    while ((cv2.waitKey(16) & 0xFF) != 27):
        if (fs_pv < 0):
            index_pv = sink_pv.get_frame_stamp()
            if (index_pv >= 0):
                fs_pv = hl2ss_mx.get_sync_frame_stamp(index_pv, hl2ss_mx.get_sync_period(producer.get_receiver(hl2ss.StreamPort.PERSONAL_VIDEO)))
        if (fs_pv >= 0):
            status_pv, index_pv, data_pv = sink_pv.get_buffered_frame(fs_pv)
            if (status_pv == hl2ss_mx.Status.OK):
                data_pv.payload = decoder_pv.decode(data_pv.payload, 'bgr24')
                cv2.imshow(window_name_pv, data_pv.payload.image)
                fs_pv += 1                
            elif (status_pv == hl2ss_mx.Status.DISCARDED):
                fs_pv = -1

    print('Stopping...')

    # Stop writers and receivers ----------------------------------------------
    for port in ports:
        writers[port].close()
        print(f'Stopped writer {hl2ss.get_port_name(port)}')

    for port in ports:
        sinks[port].detach()
        producer.stop(port)
        print(f'Stopped stream {hl2ss.get_port_name(port)}')

    # Stop subsystem ----------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, sockopt)
