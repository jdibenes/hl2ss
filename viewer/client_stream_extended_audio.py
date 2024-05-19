#------------------------------------------------------------------------------
# This script receives microphone and application audio from the HoloLens and
# plays it. The main thread receives the data, decodes it, and puts the decoded
# audio samples in a queue. A second thread gets the samples from the queue and
# plays them. Audio stream configuration is fixed to 2 channels, 48000 Hz.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import hl2ss_lnm
import hl2ss_utilities
import pyaudio
import queue
import threading
import json

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

# Microphone selection
# 1. Connect your external USB-C microphone to the HoloLens
# 2. Call hl2ss_lnm.download_devicelist_extended_audio to obtain a dictionary
#    (see audio_devices below) describing all audio capture devices
# 3. Find the value of source_index for your microphone in the dictionary
#      audio_devices[source_index]
#    The built-in microphone is included in the dictionary and can be selected
#    Only microphones that support the following configurations can be used:
#      1 or 2 channels, 48000 Hz sample rate, 16-bit PCM or 32-bit Float
#    Default device (as set in Settings > System > Sound) has source_index = -1
source_index = 0

# Audio encoding profile
profile = hl2ss.AudioProfile.AAC_24000

#------------------------------------------------------------------------------

if ((mixer_mode & hl2ss.MixerMode.QUERY) != 0):
    audio_devices = json.loads(hl2ss_lnm.download_devicelist_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO))
    print(json.dumps(audio_devices, indent=2))
    quit()


# RAW format is s16 packed, AAC decoded format is f32 planar
audio_format = pyaudio.paInt16 if (profile == hl2ss.AudioProfile.RAW) else pyaudio.paFloat32
enable = True

def pcmworker(pcmqueue):
    global enable
    global audio_format
    p = pyaudio.PyAudio()
    stream = p.open(format=audio_format, channels=hl2ss.Parameters_MICROPHONE.CHANNELS, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE, output=True)
    stream.start_stream()
    while (enable):
        stream.write(pcmqueue.get())
    stream.stop_stream()
    stream.close()

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

pcmqueue = queue.Queue()
thread = threading.Thread(target=pcmworker, args=(pcmqueue,))
listener = keyboard.Listener(on_press=on_press)
thread.start()
listener.start()

client = hl2ss_lnm.rx_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO, mixer_mode=hl2ss.extended_audio_device_mixer_mode(mixer_mode, source_index), profile=profile)
client.open()

while (enable): 
    data = client.get_next_packet()
    # RAW format is s16 packed, AAC decoded format is f32 planar
    audio = hl2ss_utilities.microphone_planar_to_packed(data.payload) if (profile != hl2ss.AudioProfile.RAW) else data.payload
    pcmqueue.put(audio.tobytes())

client.close()

enable = False
pcmqueue.put(b'')
thread.join()
listener.join()
