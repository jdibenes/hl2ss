
import socket
import pyaudio
import struct
import queue
import threading
import av

HOST = "192.168.1.15"
PORT = 3811

# bitrate
# 0: 12000 bytes/s
# 1: 16000 bytes/s
# 2: 20000 bytes/s
# 3: 24000 bytes/s
bitrate = 3

pcmqueue = queue.Queue()
tmpbuffer = bytearray()
tmpstate = 0
codec = av.CodecContext.create('aac', 'r')
resampler = av.audio.resampler.AudioResampler(format='s16', layout='stereo', rate=48000)

def pcmworker():
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels = 2, rate = 48000, output=True)
    stream.start_stream()
    while True:
        stream.write(pcmqueue.get())

threading.Thread(target=pcmworker).start()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    prev_ts = None

    s.connect((HOST, PORT))
    s.send(struct.pack('<B', bitrate))
    
    while True:
        chunk = s.recv(1024)
        if (len(chunk) == 0): break

        tmpbuffer.extend(chunk)

        while True:
            if (tmpstate == 0):
                if (len(tmpbuffer) >= 12):
                    header = struct.unpack('<QI', tmpbuffer[:12])
                    timestamp = header[0]
                    if (prev_ts and timestamp <= prev_ts):
                        print('Non-monotonic timestamps {ts} <= {prev_ts}'.format(ts=timestamp, prev_ts=prev_ts))
                    prev_ts = timestamp
                    aaclen = header[1]
                    packetlen = 12 + aaclen
                    tmpstate = 1
                else:
                    break
            elif (tmpstate == 1):
                if (len(tmpbuffer) >= packetlen):
                    adtsframe = tmpbuffer[12:packetlen]
                    packets = codec.parse(adtsframe)
                    for packet in packets:
                        for frame in codec.decode(packet):
                            for rf in resampler.resample(frame):
                               audio = rf.to_ndarray()
                               pcmqueue.put(audio.tobytes())
                    tmpbuffer = tmpbuffer[packetlen:]
                    tmpstate = 0
                else:
                    break
