#------------------------------------------------------------------------------
# Experimental Audio/Video streaming using the HoloLens 2 Device Portal API.
# Can be run without the hl2ss server running on the HoloLens, and can also be 
# run even when hl2ss (or another app) is already using the Front Camera.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_dp
import hl2ss_lnm
import hl2ss_mx
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
port = hl2ss_dp.StreamPort.LIVE

# Device Portal login
user = 'user'
password = 'pass'

# Decoded format
# Options include:
# 'bgr24'
# 'rgb24'
# 'bgra'
# 'rgba'
# 'gray8'
decoded_format = 'bgr24'

# MRC Configuration
pv = True # Enable PV video
holo = False # Enable Holograms on PV video
mic = True # Enable Microphone
loopback = False # Include application audio
render_from_camera = True # Render Holograms from PV perspective
vstab = False # Enable video stabilization
vstabbuffer = 15 # Video stabilization buffer latency in frames [0, 30]

#------------------------------------------------------------------------------

if __name__ == '__main__':
    sync_to_audio = mic or loopback

    audio_subtype     = np.float32
    audio_planar      = True
    audio_channels    = hl2ss.Parameters_MICROPHONE.CHANNELS
    audio_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE

    listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
    listener.open()

    video_buffer = hl2ss_mx.RingBuffer(30 * 10)

    player = hl2ss_utilities.audio_player(audio_subtype, audio_planar, audio_channels, audio_sample_rate)
    player.open()

    configuration = hl2ss_lnm.create_configuration_for_dp_mrc(pv, holo, mic, loopback, render_from_camera, vstab, vstabbuffer)

    client = hl2ss_lnm.rx_dp_mrc(host, port, user, password, configuration=configuration, decoded_format=decoded_format)
    client.open()

    cv2.namedWindow('Video')

    while (not listener.pressed()):
        data = client.get_next_packet()

        if (data.payload.kind == hl2ss_dp.StreamKind.AUDIO):
            print(f'got audio packet at {data.timestamp}')
            player.put(data.timestamp, data.payload.sample)
        elif (data.payload.kind == hl2ss_dp.StreamKind.VIDEO):
            print(f'got video packet at {data.timestamp} (key_frame={data.payload.key_frame})')
            video_buffer.append(data)
            frames = video_buffer.get()
            if (sync_to_audio):
                index = hl2ss_mx.get_nearest_packet(frames, player.get_timestamp(), hl2ss_mx.TimePreference.PREFER_PAST)
            else:
                index = -1
            if (index is not None):
                cv2.imshow('Video', frames[index].payload.sample)
            cv2.waitKey(1)

    client.close()
    player.close()
    listener.close()
