
from fractions import Fraction
import socket
import numpy as np
import av
import struct
from collections import deque 
import queue
import threading

HOST = "192.168.1.15"

PV_PORT = 3810
MC_PORT = 3811

# camera parameters (see pv_list.txt for supported formats)
width     = 1920
height    = 1080
framerate = 30

# video encoding profiles
# 0: h264 base
# 1: h264 main
# 2: h264 high
# 3: h265 main (HEVC)
h264profile = 3

# encoded video stream average bits per second (must be > 0)
h264bitrate = 5*1024*1024

# aac bitrate
# 0: 12000 bytes/s
# 1: 16000 bytes/s
# 2: 20000 bytes/s
# 3: 24000 bytes/s
aacbitrate = 3

# video length in seconds
videolength = 60*15

# (ok, state, buffer, timestamp, size, payload)
def unpack_payload(state_tuple):
    ok = state_tuple[0]
    state = state_tuple[1]
    buffer = state_tuple[2]
    ts = state_tuple[3]
    size = state_tuple[4]
    payload = state_tuple[5]

    ok = False
    if (state == 0):
        if (len(buffer) >= 12):
            header = struct.unpack('<QI', buffer[:12])
            ts = header[0]
            size = header[1] + 12
            state = 1
    elif (state == 1):
        if (len(buffer) >= size):
            payload = buffer[12:size]
            buffer = buffer[size:]
            state = 0
            ok = True

    return (ok, state, buffer, ts, size, payload)

container = av.open('video_test.mp4', 'w')
packetqueue = queue.PriorityQueue()
stream_h264 = container.add_stream('hevc', rate=30)
stream_aac = container.add_stream('aac', rate=48000)
codec_h264 = av.CodecContext.create('hevc', 'r')
codec_aac = av.CodecContext.create('aac', 'r')
tsfirst = None
enable = True
lock = threading.Lock()

def recv_h264():
    global tsfirst
    global enable

    state_h264 = (False, 0, bytearray(), None, None, None)
    tsqueue_h264 = deque()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket_h264:
        socket_h264.connect((HOST, PV_PORT))
        socket_h264.send(struct.pack('<HHBBIB', width, height, framerate, h264profile, h264bitrate, 0)) # PV operating mode 0

        while enable:
            chunk_h264 = socket_h264.recv(8192)
            if (len(chunk_h264) == 0):
                break
            state_h264[2].extend(chunk_h264)
            state_h264 = unpack_payload(state_h264)

            while (state_h264[0]):
                lock.acquire()
                if (not tsfirst):
                    tsfirst = state_h264[3]
                lock.release()
                tsqueue_h264.append(state_h264[3] - tsfirst)
                packets = codec_h264.parse(state_h264[5])
                for packet in packets:
                    packet.stream = stream_h264
                    packet.pts = tsqueue_h264.popleft()
                    packet.dts = packet.pts
                    packet.time_base = Fraction(1, 10*1000*1000)
                    packetqueue.put((packet.pts, packet))
                state_h264 = unpack_payload(state_h264)

def recv_aac():
    global tsfirst
    global enable

    state_aac = (False, 0, bytearray(), None, None, None)
    tsqueue_aac = deque()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket_aac:
        socket_aac.connect((HOST, MC_PORT))    
        socket_aac.send(struct.pack('<B', aacbitrate))

        while enable:
            chunk_aac = socket_aac.recv(256)
            if (len(chunk_aac) == 0):
                break
            state_aac[2].extend(chunk_aac)
            state_aac = unpack_payload(state_aac)

            while (state_aac[0]):
                lock.acquire()
                leave = not tsfirst
                lock.release()
                if (leave):
                    continue;
                tsqueue_aac.append(state_aac[3] - tsfirst)
                packets = codec_aac.parse(state_aac[5])
                for packet in packets:
                    packet.stream = stream_aac
                    packet.pts = tsqueue_aac.popleft()
                    packet.dts = packet.pts
                    packet.time_base = Fraction(1, 10*1000*1000)
                    packetqueue.put((packet.pts, packet))
                state_aac = unpack_payload(state_aac)

threading.Thread(target=recv_h264).start()
threading.Thread(target=recv_aac).start()

while enable:
    tuple = packetqueue.get()
    ts = tuple[0]
    container.mux(tuple[1])
    if (ts >= 10 * 1000 * 1000 * videolength):
        enable = False

container.close()
