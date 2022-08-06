#------------------------------------------------------------------------------
# mc capture test 
#------------------------------------------------------------------------------

import hl2ss
import pyaudio
import numpy as np
import os

# Settings --------------------------------------------------------------------

host = "192.168.1.15"
port = hl2ss.StreamPort.MICROPHONE
chunk_size = 512
profile = hl2ss.AudioProfile.AAC_24000
frames = 400
path = os.path.join('.', 'data')

#------------------------------------------------------------------------------

wr_frames = 0
rd_frames = 0

wr = hl2ss.wr_mc(path, profile)
rx = hl2ss.rx_mc(host, port, chunk_size, profile)

wr.open()
rx.open()

while (wr_frames < frames): 
    data = rx.get_next_packet()
    wr.write(data)
    wr_frames += 1

rx.close()
wr.close()

audio = pyaudio.PyAudio()
stream = audio.open(format=pyaudio.paFloat32, channels=hl2ss.Parameters_MC.CHANNELS, rate=hl2ss.Parameters_MC.SAMPLE_RATE, output=True)
stream.start_stream()

rd = hl2ss.rd_mc(path)
rd.open()

sound = np.empty((hl2ss.Parameters_MC.GROUP_SIZE * hl2ss.Parameters_MC.CHANNELS,), dtype=np.float32)

while (True):
    data = rd.read()
    if (data is None):
        break

    sound[0::2] = data.payload[0,:]
    sound[1::2] = data.payload[1,:]

    stream.write(sound.tobytes())
    print('{f}:{ts}'.format(f=rd_frames, ts=data.timestamp))
    rd_frames += 1

rd.close()

stream.stop_stream()
stream.close()

print('written frames {v}'.format(v=wr_frames))
print('read frames {v}'.format(v=rd_frames))
