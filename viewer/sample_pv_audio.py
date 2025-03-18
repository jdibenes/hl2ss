#------------------------------------------------------------------------------
# Sample audio/video player.
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
load_buffer_size = 1 # in seconds and less than full_buffer_size

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
    sink_ev = consumer.create_sink(producer, hl2ss.StreamPort.EXTENDED_AUDIO, manager, ...)

    fs_pv = sink_pv.get_attach_response()
    fs_ev = sink_ev.get_attach_response()

    # Initial audio buffering for stability -----------------------------------
    fill_count = math.ceil((load_buffer_size * hl2ss.Parameters_MICROPHONE.SAMPLE_RATE) / hl2ss.Parameters_MICROPHONE.GROUP_SIZE_AAC)
    pcm_queue = queue.Queue()
    pcm_audio_buffer = np.empty((2, 0), dtype=np.float32)
    pcm_ts_buffer = np.empty((1, 0), dtype=np.int64)
    enable = True
    presentation_clk = 0
    
    print('Buffering...')

    for _ in range(0, fill_count):
        # Audio must be read sequentially
        # using get_most_recent_frame will result in audio glitches since 
        # get_most_recent_frame repeats or drops frames as necessary
        sink_ev.acquire()
        fs_ev += 1
        _, _, data_ev = sink_ev.get_buffered_frame(fs_ev)
        pcm_queue.put(data_ev)

    print('Buffering done!')

    # Setup Multimedia Player -------------------------------------------------
    def pcmcallback(in_data, frame_count, time_info, status):
        global pcm_queue
        global pcm_audio_buffer
        global pcm_ts_buffer
        global enable
        global presentation_clk
        while (pcm_audio_buffer.shape[1] < frame_count):
            if (not enable):
                return (b'', pyaudio.paAbort)
            pcm_data_ev = pcm_queue.get()
            pcm_payload = pcm_data_ev.payload
            pcm_ts = pcm_data_ev.timestamp + (np.arange(0, pcm_payload.shape[1], 1, dtype=np.int64) * ((hl2ss.Parameters_MICROPHONE.GROUP_SIZE_AAC * hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS) // hl2ss.Parameters_MICROPHONE.SAMPLE_RATE))
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
    stream = p.open(format=pyaudio.paFloat32, channels=hl2ss.Parameters_MICROPHONE.CHANNELS, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE, output=True, stream_callback=pcmcallback)

    # Main Loop ---------------------------------------------------------------
    while (True):
        # Buffer audio packets
        sink_ev.acquire()
        fs_ev += 1
        _, _, data_ev = sink_ev.get_buffered_frame(fs_ev)
        pcm_queue.put(data_ev)
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
    sink_ev.detach()

    producer.stop(hl2ss.StreamPort.EXTENDED_AUDIO)
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
