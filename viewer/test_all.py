#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens cameras and plays it.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import multiprocessing as mp
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_mp

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Ports
ports = [
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    hl2ss.StreamPort.RM_DEPTH_AHAT,
    #hl2ss.StreamPort.RM_DEPTH_LONGTHROW
    hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
    hl2ss.StreamPort.RM_IMU_GYROSCOPE,
    hl2ss.StreamPort.RM_IMU_MAGNETOMETER,
    hl2ss.StreamPort.PERSONAL_VIDEO,
    hl2ss.StreamPort.MICROPHONE,
    hl2ss.StreamPort.SPATIAL_INPUT
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

# RM IMU parameters
imu_mode = hl2ss.StreamMode.MODE_1

# PV parameters
pv_mode = hl2ss.StreamMode.MODE_1
pv_width = 1280
pv_height = 720
pv_framerate = 30
pv_profile = hl2ss.VideoProfile.H264_MAIN
pv_bitrate = 5*1024*1024
pv_format = 'bgr24'

# Maximum number of frames in buffer
buffer_elements = 60

#------------------------------------------------------------------------------

if __name__ == '__main__':
    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    producer = hl2ss_mp.producer()
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_depth_ahat(True, host, hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss.ChunkSize.RM_DEPTH_AHAT, ahat_mode, ahat_profile, ahat_bitrate)
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, lt_mode, lt_filter)
    producer.configure_rm_imu(host, hl2ss.StreamPort.RM_IMU_ACCELEROMETER, hl2ss.ChunkSize.RM_IMU_ACCELEROMETER, imu_mode)
    producer.configure_rm_imu(host, hl2ss.StreamPort.RM_IMU_GYROSCOPE, hl2ss.ChunkSize.RM_IMU_GYROSCOPE, imu_mode)
    producer.configure_rm_imu(host, hl2ss.StreamPort.RM_IMU_MAGNETOMETER, hl2ss.ChunkSize.RM_IMU_MAGNETOMETER, imu_mode)
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, pv_mode, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate, pv_format)
    producer.configure_microphone(True, host, hl2ss.StreamPort.MICROPHONE, hl2ss.ChunkSize.MICROPHONE, hl2ss.AudioProfile.AAC_24000)
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)

    for port in ports:
        producer.initialize(port, buffer_elements)
        producer.start(port)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sinks = {}

    for port in ports:
        sinks[port] = consumer.create_sink(producer, port, manager, None)
        sinks[port].get_attach_response()

    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while (enable):
        for port in ports:
            f, data = sinks[port].get_most_recent_frame()
            if (f >= 0):
                sinks[port].get_buffered_frame(f)
            if (port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
                if (data is not None):
                    acc = hl2ss.unpack_rm_imu(data.payload)
                    print(acc.get_frame(0).temperature)

    for port in ports:
        sinks[port].detach()

    for port in ports:
        producer.stop(port)

    listener.join()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
