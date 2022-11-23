#------------------------------------------------------------------------------
# This script receives video frames and hand tracking data from the HoloLens.
# It projects the received hand joint positions onto the video frame. First,
# the intrinsics of the camera are obtained from a mode 2 transfer. Then, the 
# video stream is set to mode 1 to include the camera pose. The camera pose of
# the frame is used to project the hand joints. The streams are independent and
# run at different frame rates. Hand tracking data is sent at 60 Hz and video
# frames are sent at 30 Hz. Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import hl2ss
import hl2ss_utilities
import hl2ss_mp
import hl2ss_3dcv
import cv2

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = "192.168.1.7"

# Camera parameters
# See etc/hl2_capture_formats.txt for a list of supported formats
width     = 1920
height    = 1080
framerate = 30

# Video Encoding profiles
profile = hl2ss.VideoProfile.H264_BASE

# Encoded stream average bits per second
# Must be > 0
bitrate = 5*1024*1024

# Marker properties
radius = 5
color = (255, 255, 0)
thickness = 3

# Buffer length in seconds
buffer_length = 5

#------------------------------------------------------------------------------

if __name__ == '__main__':
    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

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

    calibration = hl2ss.download_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width, height, framerate)

    producer = hl2ss_mp.producer()
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)
    producer.initialize(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(hl2ss.StreamPort.SPATIAL_INPUT)

    consumer = hl2ss_mp.consumer()
    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, mp.Manager(), None)
    sink_si.get_attach_response()

    client_pv = hl2ss.rx_decoded_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate, 'bgr24')
    client_pv.open()

    while (enable):
        data_pv = client_pv.get_next_packet()
        data_si = sink_si.get_nearest(data_pv.timestamp)[1]

        image = data_pv.payload

        if (hl2ss.is_valid_pose(data_pv.pose) and (data_si is not None)):
            projection = hl2ss_3dcv.projection(calibration.intrinsics, hl2ss_3dcv.world_to_reference(data_pv.pose))
            si = hl2ss.unpack_si(data_si.payload)
            if (si.is_valid_hand_left()):
                project_points(image, projection, hl2ss_utilities.si_unpack_hand(si.get_hand_left()).positions, radius, color, thickness)
            if (si.is_valid_hand_right()):
                project_points(image, projection, hl2ss_utilities.si_unpack_hand(si.get_hand_right()).positions, radius, color, thickness)

        cv2.imshow('Video', image)
        cv2.waitKey(1)

    client_pv.close()
    sink_si.detach()
    producer.stop(hl2ss.StreamPort.SPATIAL_INPUT)
    listener.join()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
