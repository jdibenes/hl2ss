#------------------------------------------------------------------------------
# This script receives video from the HoloLens front RGB camera and plays it.
# The camera supports various resolutions and framerates. See
# https://github.com/jdibenes/hl2ss/blob/main/etc/pv_configurations.txt
# for a list of supported formats. The default configuration is 1080p 30 FPS. 
# The stream supports three operating modes: 0) video, 1) video + camera pose, 
# 2) query calibration (single transfer).
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss

import hl2ss_schema
import zenoh
import logging

log = logging.getLogger(__name__)

# Settings --------------------------------------------------------------------
DEFAULT_KEY = "tcn/loc/hl2/*"
# most simple zenoh config for now
conf = {"mode": "peer", "queries_default_timeout": 10000}

# Video encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

# Decoded format
# Options include:
# 'bgr24'
# 'rgb24'
# 'bgra'
# 'rgba'
# 'gray8'
decoded_format = 'bgr24'

#------------------------------------------------------------------------------


# data = hl2ss.download_calibration_pv(host, port, width, height, framerate)
# print('Calibration')
# print(f'Focal length: {data.focal_length}')
# print(f'Principal point: {data.principal_point}')
# print(f'Radial distortion: {data.radial_distortion}')
# print(f'Tangential distortion: {data.tangential_distortion}')
# print('Projection')
# print(data.projection)
# print('Intrinsics')
# print(data.intrinsics)

enable = True

logging.basicConfig(level=logging.DEBUG)

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

zenoh.init_logger()

fmt = hl2ss_schema.HL2MGR_H26xFormat(width=1280, height=720, frame_rate=15,
                                     profile=hl2ss_schema.Hololens2H26xProfile.H265Profile_Main,
                                     bitrate=20 * 1024 * 1024)

request = hl2ss_schema.HL2MGRRequest_StartPV(
    pv_format=fmt,
    enable_locataion=True,
)

with hl2ss.mgr_rpc_interface(conf, DEFAULT_KEY) as mgr:
    if not mgr.start_pv(request):
        log.error("Cannot Start Personal Video Sensor")
        exit(1)

client = hl2ss.rx_decoded_pv(conf, DEFAULT_KEY, decoded_format)
client.open()

while (enable):
    data = client.get_next_packet()

    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    print(f'Focal length: {data.payload.focal_length}')
    print(f'Principal point: {data.payload.principal_point}')

    if data.payload.image is not None:
        cv2.imshow('Video', data.payload.image)
    cv2.waitKey(1)

client.close()
listener.join()

with hl2ss.mgr_rpc_interface(conf, DEFAULT_KEY) as mgr:
    if not mgr.stop_pv():
        log.error("Cannot Stop Personal Video Sensor")