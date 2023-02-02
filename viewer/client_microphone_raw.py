#------------------------------------------------------------------------------
# This script receives uncompressed microphone audio from the HoloLens and
# plays it. The main thread receives the data, decodes it, and puts the decoded
# audio samples in a queue. A second thread gets the samples from the queue and
# plays them. Audio stream configuration is fixed to 2 channels, 48000 Hz.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import pyaudio
import queue
import threading

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
port = hl2ss.StreamPort.MICROPHONE

# Audio encoding profile
profile = hl2ss.AudioProfile.RAW

#------------------------------------------------------------------------------

enable = True

def pcmworker(pcmqueue):
    global enable
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=hl2ss.Parameters_MICROPHONE.CHANNELS, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE, output=True)
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

client = hl2ss.rx_microphone(host, port, hl2ss.ChunkSize.MICROPHONE, profile)
client.open()

while (enable): 
    data = client.get_next_packet()
    pcmqueue.put(bytes(data.payload))

client.close()

enable = False
pcmqueue.put(b'')
thread.join()
listener.join()
