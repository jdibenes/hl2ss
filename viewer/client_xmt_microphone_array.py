#------------------------------------------------------------------------------
# C++ Multithreaded Client Python Extension Test.
# Requires building the hl2ss_ulm_stream extension.
#
# This script receives RAW microphone audio from the HoloLens microphone array
# and plays it.
# Audio stream configuration is fixed to 5 channels, 48000 Hz, float32.
# Only one channel is played at a time.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import time
import hl2ss
import hl2ss_lnm
import hl2ss_mt
import pyaudio
import queue
import threading

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

# Buffer size (packets)
buffer_size = 200

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

configuration = hl2ss_mt.create_configuration(hl2ss.StreamPort.MICROPHONE)
configuration['profile'] = hl2ss.AudioProfile.RAW
configuration['level'] = hl2ss.AACLevel.L5

client = hl2ss_mt.rx_decoded_microphone(host, hl2ss.StreamPort.MICROPHONE, buffer_size, configuration)
client.open()

frame_index = -1

while (enable): 
    data = client.get_by_index(frame_index)
    if (data.status == hl2ss_mt.Status.WAIT):
        time.sleep(1/1000)
        continue
    elif (data.status == hl2ss_mt.Status.DISCARDED):
        frame_index = -1
        continue

    frame_index = data.frame_stamp + 1

    audio = data.payload[0, channel::hl2ss.Parameters_MICROPHONE.ARRAY_CHANNELS]
    pcmqueue.put(audio.tobytes())

client.close()

enable = False
pcmqueue.put(b'')
thread.join()
listener.join()
