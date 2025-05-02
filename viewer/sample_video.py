#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens cameras and plays it.
# Press esc to stop.
#------------------------------------------------------------------------------

import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mx
import hl2ss_mp
import hl2ss_utilities
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Ports
ports = [
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    #hl2ss.StreamPort.RM_DEPTH_AHAT,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
    hl2ss.StreamPort.PERSONAL_VIDEO,
    hl2ss.StreamPort.MICROPHONE,
    ]

# PV parameters
pv_width     = 760
pv_height    = 428
pv_framerate = 30

#------------------------------------------------------------------------------

if __name__ == '__main__':
    if ((hl2ss.StreamPort.RM_DEPTH_LONGTHROW in ports) and (hl2ss.StreamPort.RM_DEPTH_AHAT in ports)):
        print('Error: Simultaneous RM Depth Long Throw and RM Depth AHAT streaming is not supported.')
        quit()

    # Start PV Subsystem if PV is selected ------------------------------------
    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Configure system --------------------------------------------------------
    client_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    client_rc.open()
    # Disable buffer between sensor output and encoder input
    client_rc.ee_set_reader_buffering(False)
    # Disable buffer between encoder output and network send
    client_rc.ee_set_encoder_buffering(False)
    # Mitigate RM VLC flicker
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_LEFTFRONT,  True)
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_LEFTLEFT,   True)
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, True)
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, True)
    # RC commands that do not return a value are processed asynchronously
    # Call ee_get_application_version to ensure all commands have been processed before continuing
    client_rc.ee_get_application_version()
    client_rc.close()

    # Start streams -----------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT))
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTLEFT))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))
    producer.configure(hl2ss.StreamPort.MICROPHONE, hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE))
    
    consumer = hl2ss_mp.consumer()
    sinks = {}

    for port in ports:
        producer.initialize(port)
        producer.start(port)
        sinks[port] = consumer.get_default_sink(producer, port)
        sinks[port].get_attach_response()
        while (sinks[port].get_buffered_frame(-1)[0] != hl2ss_mx.Status.OK):
            pass
        print(f'Started {hl2ss.get_port_name(port)}')        
        
    # Create Display Map ------------------------------------------------------
    def display_pv(port, payload):
        cv2.imshow(hl2ss.get_port_name(port), payload.image)

    def display_vlc(port, payload):
        cv2.imshow(hl2ss.get_port_name(port), payload.image)

    def display_depth_lt(port, payload):
        cv2.imshow(hl2ss.get_port_name(port) + '-depth', hl2ss_3dcv.rm_depth_colormap(payload.depth, 7500))
        cv2.imshow(hl2ss.get_port_name(port) + '-ab', hl2ss_3dcv.rm_ab_normalize(payload.ab))

    def display_depth_ahat(port, payload):
        cv2.imshow(hl2ss.get_port_name(port) + '-depth', hl2ss_3dcv.rm_depth_colormap(payload.depth, 1056))
        cv2.imshow(hl2ss.get_port_name(port) + '-ab', hl2ss_3dcv.rm_ab_normalize(payload.ab))

    def display_null(port, payload):
        pass

    DISPLAY_MAP = {
        hl2ss.StreamPort.RM_VLC_LEFTFRONT     : display_vlc,
        hl2ss.StreamPort.RM_VLC_LEFTLEFT      : display_vlc,
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : display_vlc,
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : display_vlc,
        hl2ss.StreamPort.RM_DEPTH_AHAT        : display_depth_ahat,
        hl2ss.StreamPort.RM_DEPTH_LONGTHROW   : display_depth_lt,
        hl2ss.StreamPort.PERSONAL_VIDEO       : display_pv,
        hl2ss.StreamPort.MICROPHONE           : display_null,
    }

    sync_to_audio = hl2ss.StreamPort.MICROPHONE in ports

    if (sync_to_audio):
        player = hl2ss_utilities.audio_player(np.float32, True, hl2ss.Parameters_MICROPHONE.CHANNELS, hl2ss.Parameters_MICROPHONE.SAMPLE_RATE)
        player.open()
        fs_mc = -1

    for port in ports:
        if (port in [hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss.StreamPort.RM_DEPTH_LONGTHROW]):
            cv2.namedWindow(hl2ss.get_port_name(port) + '-depth')
            cv2.namedWindow(hl2ss.get_port_name(port) + '-ab')
        else:
            cv2.namedWindow(hl2ss.get_port_name(port))
    
    # Main loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        if (sync_to_audio):
            while (player.pending() < player.buffer_frames):
                status_mc, fs_mc, data_mc = sinks[hl2ss.StreamPort.MICROPHONE].get_buffered_frame(fs_mc)
                if (status_mc == hl2ss_mx.Status.OK):
                    player.put(data_mc.timestamp, data_mc.payload)
                    fs_mc += 1
                elif (status_mc == hl2ss_mx.Status.DISCARDED):
                    fs_mc = -1
                else:
                    break
            ts_mc = player.get_timestamp()

        for port in ports:
            if (port == hl2ss.StreamPort.MICROPHONE):
                continue
            if (sync_to_audio):
                _, data = sinks[port].get_nearest(ts_mc, hl2ss_mx.TimePreference.PREFER_PAST, False)
            else:
                _, data = sinks[port].get_most_recent_frame()
            if (data is not None):
                DISPLAY_MAP[port](port, data.payload)

    if (sync_to_audio):
        player.close()

    # Stop streams ------------------------------------------------------------
    for port in ports:
        if (not sinks[port].get_source_status()):
            print(f'Error during {hl2ss.get_port_name(port)} capture: ')
            print(sinks[port].get_source_string())
        sinks[port].detach()
        producer.stop(port)
        print(f'Stopped {hl2ss.get_port_name(port)}')

    # Stop PV Subsystem if PV is selected -------------------------------------
    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
