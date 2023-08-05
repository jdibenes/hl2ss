# ------------------------------------------------------------------------------
# This script receives video from one of the HoloLens sideview grayscale
# cameras and plays it. The camera resolution is 640x480 @ 30 FPS. The stream 
# supports three operating modes: 0) video, 1) video + rig pose, 2) query 
# calibration (single transfer).
# Press esc to stop.
# ------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss_imshow
import hl2ss

import zenoh
import logging

import hl2ss_schema

log = logging.getLogger(__name__)

# Settings --------------------------------------------------------------------


DEFAULT_KEY = "tcn/loc/hl2/*"
# most simple zenoh config for now
conf = {"mode": "peer", "queries_default_timeout": 10000}

# Video encoding profile
# profile = hl2ss.VideoProfile.H265_MAIN
profile = hl2ss.VideoProfile.H264_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 1 * 1024 * 1024

# ------------------------------------------------------------------------------
# data = hl2ss.download_calibration_rm_vlc(host, port)
# print('Calibration data')
# print('Image point to unit plane')
# print(data.uv2xy)
# print('Extrinsics')
# print(data.extrinsics)
# print('Undistort map')
# print(data.undistort_map)
# print('Intrinsics (undistorted only)')
# print(data.intrinsics)
# quit()

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
                                     profile=hl2ss_schema.Hololens2H26xProfile.H264Profile_High,
                                     bitrate=12 * 1024 * 1024)

request = hl2ss_schema.HL2MGRRequest_StartRM(
    enable_left_front=True,
    enable_left_left=True,
    enable_right_front=True,
    enable_right_right=True,
    vlc_format=fmt,
)

with hl2ss.mgr_rpc_interface(conf, DEFAULT_KEY) as mgr:
    if not mgr.start_rm(request):
        log.error("Cannot Start Research Mode Sensors")
        exit(1)

client = hl2ss.rx_decoded_rm_vlc(conf, DEFAULT_KEY, profile, hl2ss.StreamType.RM_VLC_LEFTFRONT)
client.open()

while (enable):
    data = client.get_next_packet()
    if data is None:
        log.error("Invalid packet.")
        continue
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    if data.payload.image is not None:
        cv2.imshow('Video', data.payload.image)
    cv2.waitKey(1)

client.close()
listener.join()
