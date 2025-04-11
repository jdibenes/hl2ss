#------------------------------------------------------------------------------
# This script receives microphone and application audio from the HoloLens and
# plays it. The main thread receives the data, decodes it, and puts the decoded
# audio samples in a queue. A second thread gets the samples from the queue and
# plays them.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import json
import hl2ss
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Mixer mode
# Options:
# hl2ss.MixerMode.MICROPHONE (microphone audio only)
# hl2ss.MixerMode.SYSTEM     (application audio only)
# hl2ss.MixerMode.BOTH       (microphone and application audio)
# hl2ss.MixerMode.QUERY      (get list of microphones)
mixer_mode = hl2ss.MixerMode.BOTH

# Gains
loopback_gain   = 1.0 # 0.0 - 5.0
microphone_gain = 1.0 # 0.0 - 5.0

# Microphone selection
# 1. Connect your external USB-C microphone to the HoloLens
# 2. Call hl2ss_lnm.download_devicelist_extended_audio to obtain a dictionary
#    (see audio_devices below) describing all audio capture devices
# 3. Find the value of device_index, source_index, and format_index for your 
#    microphone in the dictionary, which has this form:
#      audio_devices[device_index]['FrameSources'][source_index]['SupportedFormats'][format_index]
#    The built-in microphone is included in the dictionary and can be selected
#    Default device (as set in Settings > System > Sound) has device_index = -1
device_index = 0
source_index = 0
format_index = 0

# Audio encoding profile
# 1. For AAC encoded profiles (profile != hl2ss.AudioProfile.RAW)
#    The audio stream configuration is fixed to 2 channels, 48000 Hz and only
#    microphones that support the following configurations can be used:
#      1 or 2 channels, 48000 Hz sample rate, 16-bit PCM or 32-bit Float
#    In this configuration, the server selects the appropriate source_index and
#    format_index and ignores the values set by the user
#    If no suitable format is detected, the server closes the client connection
#    Finally, the server converts from 1 to 2 channels and from Float to PCM 
#    as necessary, no sample rate conversions are performed
# 2. For RAW audio profile (profile == hl2ss.AudioProfile.RAW)
#    a. If enable_passthrough=False the server selects the audio source and 
#       format (ignoring source_index and format_index), and converts audio as 
#       described above
#    b. If enable_passthrough=True the user selects the audio source 
#       (source_index) and format (format_index) and audio is streamed as-is,
#       no transforms are performed
#       There are no restrictions on number of channels or sample rate, however
#       the audio mixing effect for application audio is not compatible with 
#       all configurations, so set disable_effect=False in those cases
#       Exclusive access can be set with shared=False
#       The audio format exposed by the device can change for different values
#       of media_category and audio_raw
#       For example, for the built-in microphone:
#         media_category=hl2ss.MediaCategory.Media,  audio_raw=False  =>   2 channels, 48000 Hz, Float
#         media_category=hl2ss.MediaCategory.Speech, audio_raw=False  =>   1 channel,  16000 Hz, Float
#         media_category=[Don't Care],               audio_raw=True   =>  11 channels, 48000 Hz, Float (Microphone Array)
#       The server closes the client connection on invalid configurations
profile = hl2ss.AudioProfile.AAC_24000

enable_passthrough = False
disable_effect     = True
shared             = True
media_category     = hl2ss.MediaCategory.Media
audio_raw          = False

# Audio configuration for passthrough
# Set these to match your microphone format if using
# profile=hl2ss.AudioProfile.RAW and enable_passthrough=True
audio_passthrough_subtype     = None
audio_passthrough_planar      = False
audio_passthrough_channels    = None
audio_passthrough_sample_rate = None

#------------------------------------------------------------------------------

if __name__ == '__main__':
    level = hl2ss.AACLevel.L2 if (profile != hl2ss.AudioProfile.RAW) else hl2ss.extended_audio_raw_configuration(media_category, shared, audio_raw, disable_effect, enable_passthrough)

    if (mixer_mode == hl2ss.MixerMode.QUERY):
        audio_devices = json.loads(hl2ss_lnm.download_devicelist_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO, profile=profile, level=level))
        print(json.dumps(audio_devices, indent=2))
        quit()


    if (profile != hl2ss.AudioProfile.RAW):
        audio_subtype     = np.float32
        audio_planar      = True
        audio_channels    = hl2ss.Parameters_MICROPHONE.CHANNELS
        audio_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
    elif (not enable_passthrough):
        audio_subtype     = np.int16
        audio_planar      = False
        audio_channels    = hl2ss.Parameters_MICROPHONE.CHANNELS
        audio_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
    else:
        audio_subtype     = audio_passthrough_subtype
        audio_planar      = audio_passthrough_planar
        audio_channels    = audio_passthrough_channels
        audio_sample_rate = audio_passthrough_sample_rate

    listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
    listener.open()

    # audio_player only supports 1 or 2 channels
    player_max_channels = 2
    player_channels = audio_channels if (audio_channels <= player_max_channels) else 1
    player = hl2ss_utilities.audio_player(audio_subtype, audio_planar, player_channels, audio_sample_rate)
    player.open()

    mode = hl2ss.extended_audio_device_mixer_mode(mixer_mode, device_index, source_index, format_index)

    client = hl2ss_lnm.rx_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO, mixer_mode=mode, loopback_gain=loopback_gain, microphone_gain=microphone_gain, profile=profile, level=level)
    client.open()

    while (not listener.pressed()): 
        data = client.get_next_packet()
        # Passthrough data is returned as int8 and should be cast to subtype
        if (data.payload.dtype == np.int8):
            data.payload = data.payload.view(audio_subtype)
        # Play first channel for streams with more than 2 channels
        if (audio_channels > player_max_channels):
            data.payload = data.payload[0:1, :] if (audio_planar) else data.payload[:, 0::audio_channels]
        player.put(data.timestamp, data.payload)

    client.close()
    player.close()
    listener.close()
