#------------------------------------------------------------------------------
# This script receives microphone audio from the HoloLens and plays it. The 
# main thread receives the data, decodes it, and puts the decoded audio samples
# in a queue. A second thread gets the samples from the queue and plays them.
# Audio stream configuration is fixed to 2 channels, 48000 Hz.
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

# Audio encoding profile
profile = hl2ss.AudioProfile.AAC_24000

#------------------------------------------------------------------------------

if __name__ == '__main__':
    audio_subtype     = np.int16 if (profile == hl2ss.AudioProfile.RAW) else np.float32
    audio_planar      = profile != hl2ss.AudioProfile.RAW
    audio_channels    = hl2ss.Parameters_MICROPHONE.CHANNELS
    audio_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE

    listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
    listener.open()

    player = hl2ss_utilities.audio_player(audio_subtype, audio_planar, audio_channels, audio_sample_rate)
    player.open()

    client = hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE, profile=profile)
    client.open()

    while (not listener.pressed()):
        data = client.get_next_packet()
        player.put(data.timestamp, data.payload)

    client.close()
    player.close()
    listener.close()
