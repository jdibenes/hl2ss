#------------------------------------------------------------------------------
# This script records video from the HoloLens front RGB camera and audio from
# the HoloLens microphone to a mp4 file.
#------------------------------------------------------------------------------

import hl2ss
from fractions import Fraction
import av
import queue
import threading

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Ports
port_pv = hl2ss.StreamPort.PERSONAL_VIDEO
port_mc = hl2ss.StreamPort.MICROPHONE

# Camera parameters
# See etc/hl2_capture_formats.txt for a list of supported formats.
width     = 1920
height    = 1080
framerate = 30

# Video encoding profile
video_profile = hl2ss.VideoProfile.H265_MAIN

# Encoded video stream average bits per second
# Must be > 0
video_bitrate = 5*1024*1024

# Audio encoding profile
audio_profile = hl2ss.AudioProfile.AAC_24000

# Video length in seconds
video_length = 60*10

#------------------------------------------------------------------------------

video_codec_name = hl2ss.get_video_codec_name(video_profile)
audio_codec_name = hl2ss.get_audio_codec_name(audio_profile)

container = av.open('video.mp4', 'w')
stream_video = container.add_stream(video_codec_name, rate=framerate)
stream_audio = container.add_stream(audio_codec_name, rate=48000)
codec_video = av.CodecContext.create(video_codec_name, 'r')
codec_audio = av.CodecContext.create(audio_codec_name, 'r')

packetqueue = queue.PriorityQueue()
lock = threading.Lock()

tsfirst = None
enable = True

def recv_pv():
    global tsfirst
    global enable

    client = hl2ss.connect_client_pv(host, port_pv, 8192, hl2ss.StreamMode.MODE_0, width, height, framerate, video_profile, video_bitrate)
    while (enable):
        data = client.get_next_packet()
        lock.acquire()
        if (not tsfirst):
            tsfirst = data.timestamp
        lock.release()
        packets = codec_video.parse(data.payload)
        for packet in packets:
            packet.stream = stream_video
            packet.pts = data.timestamp - tsfirst
            packet.dts = packet.pts
            packet.time_base = Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)
            packetqueue.put((packet.pts, packet))

def recv_mc():
    global tsfirst
    global enable

    client = hl2ss.connect_client_mc(host, port_mc, 256, audio_profile)
    while (enable):
        data = client.get_next_packet()
        lock.acquire()
        leave = tsfirst is None
        lock.release()
        if (leave):
            continue
        packets = codec_audio.parse(data.payload)
        for packet in packets:
            packet.stream = stream_audio
            packet.pts = data.timestamp - tsfirst
            packet.dts = packet.pts
            packet.time_base = Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)
            packetqueue.put((packet.pts, packet))

thread_pv = threading.Thread(target=recv_pv)
thread_mc = threading.Thread(target=recv_mc)

thread_pv.start()
thread_mc.start()

while enable:
    tuple = packetqueue.get()
    ts = tuple[0]
    container.mux(tuple[1])
    if (ts >= (hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS * video_length)):
        enable = False

container.close()

thread_pv.join()
thread_mc.join()
