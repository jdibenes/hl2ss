#------------------------------------------------------------------------------
# This script receives AAC encoded microphone audio from the HoloLens and
# plays it. The main thread receives the data, decodes it, and puts the decoded
# audio samples in a queue. A second thread gets the samples from the queue and
# plays them. Audio stream configuration is fixed to 2 channels, 48000 Hz.
#------------------------------------------------------------------------------

import hl2ss
import pyaudio
import queue
import threading
import av

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port
port = hl2ss.StreamPort.MICROPHONE

# Audio encoding profile
profile = hl2ss.AudioProfile.AAC_24000

#------------------------------------------------------------------------------

enable = True
pcmqueue = queue.Queue()
codec = av.CodecContext.create(hl2ss.get_audio_codec_name(profile), 'r')
resampler = av.audio.resampler.AudioResampler(format='s16', layout=hl2ss.Parameters_MC.LAYOUT, rate=hl2ss.Parameters_MC.SAMPLE_RATE)

def pcmworker():
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=hl2ss.Parameters_MC.CHANNELS, rate=hl2ss.Parameters_MC.SAMPLE_RATE, output=True)
    stream.start_stream()
    while enable:
        stream.write(pcmqueue.get())
    stream.stop_stream()
    stream.close()

thread = threading.Thread(target=pcmworker)
thread.start()

client = hl2ss.connect_client_mc(host, port, 512, profile)

try:
    while True: 
        data = client.get_next_packet()
        for packet in codec.parse(data.payload):
            # Decoded audio format is float32
            for frame in codec.decode(packet): 
                # Convert to int16
                for audio in resampler.resample(frame): 
                    pcmqueue.put(audio.to_ndarray().tobytes())
except:
    pass

client.close()

enable = False
pcmqueue.put(b'')
thread.join()
