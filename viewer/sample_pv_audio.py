#------------------------------------------------------------------------------
# Sample audio/video player (Experimental).
# Press ESC to stop.
#------------------------------------------------------------------------------

import numpy as np
import multiprocessing as mp
import queue
import math
import pyaudio
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Video parameters
width = 1280
height = 720
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
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=width, height=height, framerate=framerate))
    producer.configure(hl2ss.StreamPort.EXTENDED_AUDIO, hl2ss_lnm.rx_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO, mixer_mode=mixer_mode))
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, math.ceil(full_buffer_size * framerate))
    producer.initialize(hl2ss.StreamPort.EXTENDED_AUDIO, math.ceil((full_buffer_size * hl2ss.Parameters_MICROPHONE.SAMPLE_RATE) / hl2ss.Parameters_MICROPHONE.GROUP_SIZE_AAC))
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.EXTENDED_AUDIO)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_ea = consumer.create_sink(producer, hl2ss.StreamPort.EXTENDED_AUDIO, manager, ...)

    fs_pv = sink_pv.get_attach_response()
    fs_ea = sink_ea.get_attach_response()

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
        # using get_most_recent_frame will result in audio glitches since 
        # get_most_recent_frame repeats or drops frames as necessary
        sink_ea.acquire()
        fs_ea += 1
        _, _, data_ea = sink_ea.get_buffered_frame(fs_ea)
        pcm_queue.put(data_ea)
        if (not enable):
            break
        # Coarse audio/video synchronization based on audio frame timestamps
        if (presentation_clk > 0):
            # Sync to audio
            _, data_pv = sink_pv.get_nearest(presentation_clk, hl2ss_mp.TimePreference.PREFER_PAST)
            if (data_pv is not None):
                cv2.imshow('Video', data_pv.payload.image)
        # Stop if ESC is pressed
        enable = (cv2.waitKey(1) & 0xFF) != 27

    # Close PyAudio Stream ----------------------------------------------------
    stream.close()

    # Stop PV and Extended Audio streams --------------------------------------
    sink_pv.detach()
    sink_ea.detach()

    producer.stop(hl2ss.StreamPort.EXTENDED_AUDIO)
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
