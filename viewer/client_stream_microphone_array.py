#------------------------------------------------------------------------------
# This script receives RAW microphone audio from the HoloLens microphone array
# and plays it.
# Audio stream configuration is fixed to 5 channels, 48000 Hz, float32.
# Only one channel is played at a time.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import hl2ss
import hl2ss_lnm
import hl2ss_utilities
import pyaudio
import queue
import threading

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Channel (0-4)
# 0 top left?
# 1 top center?
# 2 top right?
# 3 bottom left?
# 4 bottom right?
channel = 4

#------------------------------------------------------------------------------

audio_format = pyaudio.paFloat32
enable = True

def pcmworker(pcmqueue):
    global enable
    global audio_format
    p = pyaudio.PyAudio()
    stream = p.open(format=audio_format, channels=1, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE, output=True)
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

client = hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE, profile=hl2ss.AudioProfile.RAW, level=hl2ss.AACLevel.L5)
client.open()

while (enable): 
    data = client.get_next_packet()
    samples = np.frombuffer(data.payload, dtype=np.float32)
    audio = samples[channel::5].copy()
    pcmqueue.put(audio.tobytes())

client.close()

enable = False
pcmqueue.put(b'')
thread.join()
listener.join()
