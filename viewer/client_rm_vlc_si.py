#------------------------------------------------------------------------------
# This script receives video frames and hand tracking data from the HoloLens.
# It projects the received hand joint positions onto the video frame. First,
# the camera model is loaded from the calibration folder. Then, the video
# stream is set to mode 1 to include the camera pose. The camera pose of the 
# frame is used to project the hand joints. The streams are independent and run
# at different frame rates. Hand tracking data is sent at 60 Hz and video
# frames are sent at 30 Hz. Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_utilities
import hl2ss_3dcv
import hl2ss_mp

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = "192.168.1.7"

# Port
port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

# Video Encoding profiles
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 1*1024*1024

# Marker properties
radius = 5
color = (255, 255, 0)
thickness = 3

# Buffer length in seconds
buffer_length = 5

#------------------------------------------------------------------------------

if __name__ == '__main__':
    enable = True

    def project_points(image, P, points, radius, color, thickness):
        for x, y in hl2ss_3dcv.project_to_image(hl2ss_3dcv.to_homogeneous(points), P)[0]:
            cv2.circle(image, (int(x), (int(y))), radius, color, thickness)

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    model_vlc = hl2ss.download_calibration_rm_vlc(host, port)

    producer = hl2ss_mp.producer()
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)
    producer.initialize(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(hl2ss.StreamPort.SPATIAL_INPUT)

    consumer = hl2ss_mp.consumer()
    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, mp.Manager(), None)
    sink_si.get_attach_response()

    client_vlc = hl2ss.rx_decoded_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_1, profile, bitrate)
    client_vlc.open()

    while (enable):
        data_vlc = client_vlc.get_next_packet()
        data_si = sink_si.get_nearest(data_vlc.timestamp)[1]

        image = cv2.remap(data_vlc.payload, model_vlc.undistort_map[:, :, 0], model_vlc.undistort_map[:, :, 1], cv2.INTER_LINEAR)
        image = np.dstack((image, image, image))

        if (hl2ss.is_valid_pose(data_vlc.pose) and (data_si is not None)):
            projection = hl2ss_3dcv.projection(model_vlc.intrinsics, hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(model_vlc.extrinsics))
            si = hl2ss.unpack_si(data_si.payload)
            if (si.is_valid_hand_left()):
                project_points(image, projection, hl2ss_utilities.si_unpack_hand(si.get_hand_left()).positions, radius, color, thickness)
            if (si.is_valid_hand_right()):
                project_points(image, projection, hl2ss_utilities.si_unpack_hand(si.get_hand_right()).positions, radius, color, thickness)

        cv2.imshow('Video', image)
        cv2.waitKey(1)

    client_vlc.close()
    sink_si.detach()
    producer.stop(hl2ss.StreamPort.SPATIAL_INPUT)
    listener.join()
