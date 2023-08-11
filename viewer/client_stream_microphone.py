#------------------------------------------------------------------------------
# This script receives microphone audio from the HoloLens and plays it. The 
# main thread receives the data, decodes it, and puts the decoded audio samples
# in a queue. A second thread gets the samples from the queue and plays them.
# Audio stream configuration is fixed to 2 channels, 48000 Hz.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import hl2ss_lnm
import hl2ss_utilities
import pyaudio
import queue
import threading

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Audio encoding profile
profile = hl2ss.AudioProfile.AAC_24000

#------------------------------------------------------------------------------

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

client = hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE, profile=profile)
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
