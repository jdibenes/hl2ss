#------------------------------------------------------------------------------
# Recording example. Data is recorded to binary files. See simple player for
# how to extract recorded data.
# Press space to start recording.
# Press esc to stop recording.
#------------------------------------------------------------------------------

from datetime import datetime
from pynput import keyboard

import os
import time
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

# Ports to record
ports = [
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    #hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    #hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    #hl2ss.StreamPort.RM_DEPTH_AHAT,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
    #hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
    #hl2ss.StreamPort.RM_IMU_GYROSCOPE,
    #hl2ss.StreamPort.RM_IMU_MAGNETOMETER,
    hl2ss.StreamPort.PERSONAL_VIDEO,
    hl2ss.StreamPort.MICROPHONE,
    #hl2ss.StreamPort.SPATIAL_INPUT,
    #hl2ss.StreamPort.EXTENDED_EYE_TRACKER,
    #hl2ss.StreamPort.EXTENDED_AUDIO,
    #hl2ss.StreamPort.EXTENDED_VIDEO,
    #hl2ss.StreamPort.EXTENDED_DEPTH,
    ]

# Encoding profile for video streams
video_profile = hl2ss.VideoProfile.H264_MAIN

# RM Depth AHAT parameters
profile_z = hl2ss.DepthProfile.SAME

# PV parameters
pv_width = 760
pv_height = 428
pv_framerate = 30

# EET parameters
# fps 30, 60, or 90
eet_fps = 30

# Extended Audio parameters
ea_mixer_mode = hl2ss.MixerMode.BOTH
ea_device_index = 0
ea_source_index = 0
ea_format_index = 0
ea_loopback_gain = 1.0
ea_microphone_gain = 1.0

# Extended Video parameters
ev_group_index = 2
ev_source_index = 0
ev_profile_index = 0
ev_width = 1280
ev_height = 720
ev_framerate = 30

# Extended Depth parameters
ez_group_index = 0
ez_source_index = 0
ez_profile_index = 0
ez_media_index = 15
ez_stride_mask = 63

# User data
user_data = 'created by hl2ss simple recorder'.encode()

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Check selected ports ----------------------------------------------------
    if (len(ports) <= 0):
        print('No ports selected')
        quit()

    if ((hl2ss.StreamPort.RM_DEPTH_LONGTHROW in ports) and (hl2ss.StreamPort.RM_DEPTH_AHAT in ports)):
        print('Error: Simultaneous RM Depth Long Throw and RM Depth AHAT streaming is not supported.')
        quit()

    # Wait for start signal ---------------------------------------------------
    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()

    print('Press space to start recording...')
    while (not listener.pressed()):
        time.sleep(1/60)
    print('Preparing...')

    listener.close()

    # Generate output filenames -----------------------------------------------
    now    = datetime.now()
    folder = os.path.join(path, f'DS-{now.year:04d}-{now.month:02d}-{now.day:02d}-{now.hour:02d}-{now.minute:02d}-{now.second:02d}')

    # Create output folder
    # Fail if folder exists to avoid overwriting previous data, if any
    os.makedirs(folder)

    filenames = {port : os.path.join(folder, f'{hl2ss.get_port_name(port)}.bin') for port in ports}

    # Start subsystems if selected --------------------------------------------
    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    if (hl2ss.StreamPort.EXTENDED_VIDEO in ports):
        hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO, global_opacity=ev_group_index, output_width=ev_source_index, output_height=ev_profile_index)

    if (hl2ss.StreamPort.EXTENDED_DEPTH in ports):
        hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH, global_opacity=ez_group_index, output_width=ez_source_index, output_height=ez_profile_index)

    # Configure system --------------------------------------------------------
    client_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    client_rc.open()
    # Configure PV camera
    client_rc.pv_wait_for_subsystem(True)
    client_rc.pv_set_focus(hl2ss.PV_FocusMode.Manual, hl2ss.PV_AutoFocusRange.Normal, hl2ss.PV_ManualFocusDistance.Infinity, 3000, hl2ss.PV_DriverFallback.Disable)
    client_rc.pv_set_exposure(hl2ss.PV_ExposureMode.Manual, 16666)
    client_rc.pv_set_exposure_priority_video(hl2ss.PV_ExposurePriorityVideo.Disabled)
    client_rc.pv_set_white_balance_preset(hl2ss.PV_ColorTemperaturePreset.Flash)
    client_rc.pv_set_video_temporal_denoising(hl2ss.PV_VideoTemporalDenoisingMode.Off)
    client_rc.pv_set_backlight_compensation(hl2ss.PV_BacklightCompensationState.Disable)
    # Disable buffering
    client_rc.ee_set_reader_buffering(False)
    client_rc.ee_set_encoder_buffering(False)
    # Enable RM VLC patch
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_LEFTFRONT,  True)
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_LEFTLEFT,   True)
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, True)
    client_rc.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, True)
    # Wait for command completion
    client_rc.ee_get_application_version()
    client_rc.close()
    
    # Start receivers ---------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, profile=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT, profile_z=profile_z, profile_ab=video_profile, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_IMU_ACCELEROMETER, hl2ss_lnm.rx_rm_imu(host, hl2ss.StreamPort.RM_IMU_ACCELEROMETER, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_IMU_GYROSCOPE, hl2ss_lnm.rx_rm_imu(host, hl2ss.StreamPort.RM_IMU_GYROSCOPE, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_IMU_MAGNETOMETER, hl2ss_lnm.rx_rm_imu(host, hl2ss.StreamPort.RM_IMU_MAGNETOMETER, decoded=False))
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, profile=video_profile, decoded_format=None))
    producer.configure(hl2ss.StreamPort.MICROPHONE, hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE, decoded=False))
    producer.configure(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT, decoded=False))
    producer.configure(hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss_lnm.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, fps=eet_fps, decoded=False))
    producer.configure(hl2ss.StreamPort.EXTENDED_AUDIO, hl2ss_lnm.rx_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO, mixer_mode=hl2ss.extended_audio_device_mixer_mode(ea_mixer_mode, ea_device_index, ea_source_index, ea_format_index), loopback_gain=ea_loopback_gain, microphone_gain=ea_microphone_gain, decoded=False))
    producer.configure(hl2ss.StreamPort.EXTENDED_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO, width=ev_width, height=ev_height, framerate=ev_framerate, profile=video_profile, decoded_format=None))
    producer.configure(hl2ss.StreamPort.EXTENDED_DEPTH, hl2ss_lnm.rx_extended_depth(host, hl2ss.StreamPort.EXTENDED_DEPTH, media_index=ez_media_index, stride_mask=ez_stride_mask, decoded=False))

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

    listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
    listener.open()

    print('Press esc to stop recording...')
    while (not listener.pressed()):
        time.sleep(1/60)
    print('Stopping...')

    listener.close()

    # Stop writers and receivers ----------------------------------------------
    for port in ports:
        writers[port].close()
        print(f'Stopped writer {hl2ss.get_port_name(port)}')

    for port in ports:
        sinks[port].detach()
        producer.stop(port)
        print(f'Stopped stream {hl2ss.get_port_name(port)}')

    # Stop subsystems if selected ---------------------------------------------
    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    if (hl2ss.StreamPort.EXTENDED_VIDEO in ports):
        hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO)

    if (hl2ss.StreamPort.EXTENDED_DEPTH in ports):
        hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH)
