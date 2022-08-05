# Workaround for the opencv imshow issue in some Ubuntu installations
#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens front RGB camera and
# plays it. The camera support various resolutions and framerates. See
# etc/hl2_capture_formats.txt for a list of supported formats. The default
# configuration is 1080p 30 FPS. The stream supports three operating modes:
# 0) video, 1) video + camera pose, 2) query calibration (single transfer).
#------------------------------------------------------------------------------

import hl2ss
import tkinter
import av
from PIL import ImageTk, Image

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port
port = hl2ss.StreamPort.PERSONAL_VIDEO

# Operating mode
# 0: video
# 1: video + camera pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Camera parameters
width     = 1280
height    = 720
framerate = 30

# Video encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 5*1024*1024

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_pv(host, port, width, height, framerate, profile, bitrate)
    print('Calibration')
    print(data.focal_length)
    print(data.principal_point)
    print(data.radial_distortion)
    print(data.tangential_distortion)
    print(data.projection)
    quit()

root = tkinter.Tk()
label = tkinter.Label(root)
label.place(x=0, y=0)

codec = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')
pose_printer = hl2ss.pose_printer(60)
fps_counter = hl2ss.framerate_counter(60)
glitch_detector = hl2ss.continuity_analyzer(hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS / framerate)
client = hl2ss.connect_client_pv(host, port, 4096, mode, width, height, framerate, profile, bitrate)

try:
    while True:
        data = client.get_next_packet()
        timestamp = data.timestamp
        for packet in codec.parse(data.payload):
            for frame in codec.decode(packet):
                image = frame.to_ndarray(format='rgb24')

                glitch_detector.push(timestamp)
                fps_counter.push()
                pose_printer.push(timestamp, data.pose)

                test = ImageTk.PhotoImage(image=Image.fromarray(image))
                label.configure(image=test)
                
                root.update_idletasks()
                root.update()
except:
    pass

client.close()
