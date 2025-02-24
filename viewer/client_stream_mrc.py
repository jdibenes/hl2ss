#------------------------------------------------------------------------------
# Experimental Audio/Video streaming using the HoloLens 2 Device Portal API.
# Can be run without the hl2ss server running on the HoloLens, and can also be 
# run even when hl2ss (or another app) is already using the Front Camera.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_dp
import hl2ss_lnm
import hl2ss_utilities
import pyaudio
import queue
import threading

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

enable = True

def pcmworker(pcmqueue):
    global enable
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paFloat32, channels=hl2ss.Parameters_MICROPHONE.CHANNELS, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE, output=True)
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

configuration = hl2ss_dp.create_configuration_for_mrc(pv, holo, mic, loopback, render_from_camera, vstab, vstabbuffer)

client = hl2ss_lnm.rx_mrc(host, port, user, password, configuration=configuration, decoded_format=decoded_format)
client.open()

while (enable):
    data = client.get_next_packet()
    if (data.payload.kind == hl2ss_dp.StreamKind.AUDIO):
        print(f'got audio packet at {data.timestamp}')
        audio = hl2ss_utilities.microphone_planar_to_packed(data.payload.sample)
        pcmqueue.put(audio.tobytes())
    elif (data.payload.kind == hl2ss_dp.StreamKind.VIDEO):
        print(f'got video packet at {data.timestamp} (key_frame={data.payload.key_frame})')
        cv2.imshow('video', data.payload.sample)
        cv2.waitKey(1)

client.close()
pcmqueue.put(b'')
thread.join()
listener.join()
