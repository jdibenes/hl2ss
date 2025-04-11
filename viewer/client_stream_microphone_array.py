#------------------------------------------------------------------------------
# This script receives RAW microphone audio from the HoloLens microphone array
# and plays it.
# Audio stream configuration is fixed to 5 channels, 48000 Hz, float32.
# All channels are received but only one channel is played at a time.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import hl2ss
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Channel
# Options:
# hl2ss.Parameters_MICROPHONE.ARRAY_TOP_LEFT
# hl2ss.Parameters_MICROPHONE.ARRAY_TOP_CENTER
# hl2ss.Parameters_MICROPHONE.ARRAY_TOP_RIGHT
# hl2ss.Parameters_MICROPHONE.ARRAY_BOTTOM_LEFT
# hl2ss.Parameters_MICROPHONE.ARRAY_BOTTOM_RIGHT
channel = hl2ss.Parameters_MICROPHONE.ARRAY_TOP_LEFT

#------------------------------------------------------------------------------

if __name__ == '__main__':
    audio_subtype     = np.float32
    audio_planar      = False
    audio_channels    = 1 # audio_player only supports 1 or 2 channels
    audio_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE

    listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
    listener.open()

    player = hl2ss_utilities.audio_player(audio_subtype, audio_planar, audio_channels, audio_sample_rate)
    player.open()

    client = hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE, profile=hl2ss.AudioProfile.RAW, level=hl2ss.AACLevel.L5)
    client.open()

    while (not listener.pressed()): 
        data = client.get_next_packet()
        # Extract channel to play
        data.payload = data.payload[:, channel::hl2ss.Parameters_MICROPHONE.ARRAY_CHANNELS]
        player.put(data.timestamp, data.payload)

    client.close()
    player.close()
    listener.close()
