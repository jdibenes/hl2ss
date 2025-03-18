#------------------------------------------------------------------------------
# Low-Latency C++ Multithreaded Client Python Extension Test.
# Requires building the hl2ss_ulm_stream extension.
#
# Sample audio/video player (Experimental).
# Press ESC to stop.
#------------------------------------------------------------------------------

import numpy as np
import time
import queue
import math
import pyaudio
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mt
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Video parameters
width = 1920
height = 1080
framerate = 30

# Audio parameters
mixer_mode = hl2ss.MixerMode.MICROPHONE

# Buffer parameters
full_buffer_size = 5 # in seconds

#------------------------------------------------------------------------------

if __name__ == "__main__":
    # Start PV Subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Start PV and Extended Audio streams -------------------------------------
    configuration_pv = hl2ss_mt.create_configuration(hl2ss.StreamPort.PERSONAL_VIDEO)
    configuration_pv['width'] = width
    configuration_pv['height'] = height
    configuration_pv['framerate'] = framerate

    configuration_ea = hl2ss_mt.create_configuration(hl2ss.StreamPort.EXTENDED_AUDIO)
    configuration_ea['mixer_mode'] = mixer_mode

    client_pv = hl2ss_mt.rx_decoded_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, math.ceil(full_buffer_size * framerate), configuration_pv)
    client_ea = hl2ss_mt.rx_decoded_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO, math.ceil((full_buffer_size * hl2ss.Parameters_MICROPHONE.SAMPLE_RATE) / hl2ss.Parameters_MICROPHONE.GROUP_SIZE_AAC), configuration_ea)
    client_pv.open()
    client_ea.open()

    fs_pv = -1
    fs_ea = -1

    # Setup Multimedia Player -------------------------------------------------
    pcm_queue = queue.Queue()
    pcm_audio_buffer = np.empty((2, 0), dtype=np.float32)
    pcm_ts_buffer = np.empty((1, 0), dtype=np.int64)
    enable = True
    presentation_clk = 0
    
    def pcm_callback(in_data, frame_count, time_info, status):
        global pcm_queue
        global pcm_audio_buffer
        global pcm_ts_buffer
        global enable
        global presentation_clk
        while (pcm_audio_buffer.shape[1] < frame_count):
            if (not enable):
                return (b'', pyaudio.paAbort)
            pcm_data_ea = pcm_queue.get()
            pcm_payload = pcm_data_ea.payload
            pcm_group_size = pcm_payload.shape[1]
            pcm_ts = pcm_data_ea.timestamp + (np.arange(0, pcm_group_size, 1, dtype=np.int64) * ((pcm_group_size * hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS) // hl2ss.Parameters_MICROPHONE.SAMPLE_RATE))
            pcm_audio_buffer = np.hstack((pcm_audio_buffer, pcm_payload))
            pcm_ts_buffer = np.hstack((pcm_ts_buffer, pcm_ts.reshape((1, -1))))
        out_data = hl2ss_utilities.microphone_planar_to_packed(pcm_audio_buffer[:, 0:frame_count]).tobytes()
        presentation_clk = pcm_ts_buffer[0,0]
        pcm_audio_buffer = pcm_audio_buffer[:, frame_count:]
        pcm_ts_buffer = pcm_ts_buffer[:, frame_count:]
        return (out_data, pyaudio.paContinue if (enable) else pyaudio.paAbort)
    
    cv2.namedWindow('Video')
    
    # Open PyAudio Stream -----------------------------------------------------
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paFloat32, channels=hl2ss.Parameters_MICROPHONE.CHANNELS, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE, output=True, stream_callback=pcm_callback)

    # Main Loop ---------------------------------------------------------------
    while (True):
        # Buffer audio packets
        # audio must be read sequentially
        data_ea = client_ea.get_by_index(fs_ea)
        if (data_ea.status == hl2ss_mt.Status.WAIT):
            time.sleep(1/1000)
            continue
        elif (data_ea.status == hl2ss_mt.Status.DISCARDED):
            fs_ea = -1
            continue
        fs_ea = data_ea.frame_stamp + 1
        pcm_queue.put(data_ea)
        if (not enable):
            break
        # Coarse audio/video synchronization based on audio frame timestamps
        if (presentation_clk > 0):
            # Sync to audio
            data_pv = client_pv.get_by_timestamp(presentation_clk, hl2ss_mt.TimePreference.PREFER_PAST)
            if (data_pv.status == hl2ss_mt.Status.OK):
                cv2.imshow('Video', data_pv.payload.image)
        # Stop if ESC is pressed
        enable = (cv2.waitKey(1) & 0xFF) != 27
    
    # Close PyAudio Stream ----------------------------------------------------
    stream.close()

    # Stop PV and Extended Audio streams --------------------------------------
    client_ea.close()
    client_pv.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
