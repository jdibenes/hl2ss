#------------------------------------------------------------------------------
# This script receives AAC encoded microphone audio from the HoloLens and
# plays it. The main thread receives the data, decodes it, and puts the decoded
# audio samples in a queue. A second thread gets the samples from the queue and
# plays them. Audio stream configuration is fixed to 2 channels, 48000 Hz.

import hl2ss
import pyaudio
import queue
import threading
import av

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port number
port = hl2ss.StreamPort.MICROPHONE

# Audio encoding profile
profile = hl2ss.AudioProfile.AAC_24000

# Maximum number of bytes to read from the socket buffer per step
# Use an appropriate power-of-two value
chunk_size = 512

#------------------------------------------------------------------------------

enable = True
pcmqueue = queue.Queue()

def pcmworker():
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=2, rate=48000, output=True)
    stream.start_stream()
    while enable:
        stream.write(pcmqueue.get())
    stream.stop_stream()
    stream.close()

threading.Thread(target=pcmworker).start()

client = hl2ss.gatherer()
codec = av.CodecContext.create(hl2ss.get_audio_codec_name(profile), 'r')
resampler = av.audio.resampler.AudioResampler(format='s16', layout='stereo', rate=48000)

client.open(host, port, chunk_size, hl2ss.StreamMode.MODE_0)
client.configure(hl2ss.create_configuration_for_audio(profile))

while True: 
    data = client.get_next_packet()
    packets = codec.parse(data.payload)
    for packet in packets:
        # Decoded audio format is float32
        for frame in codec.decode(packet): 
            # Convert to int16
            for audio in resampler.resample(frame): 
                pcmqueue.put(audio.to_ndarray().tobytes()) 
