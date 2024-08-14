#------------------------------------------------------------------------------
# This script performs stereo rectification for a pair of RM VLC cameras.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = '192.168.1.7'

# Ports
port_left  = hl2ss.StreamPort.RM_VLC_LEFTFRONT
port_right = hl2ss.StreamPort.RM_VLC_RIGHTFRONT

# Calibration folder (must exist but can be empty)
calibration_path = '../calibration'

# Buffer size in seconds
buffer_size = 10

# Undistort rectify map shape
shape = hl2ss.Parameters_RM_VLC.SHAPE

# Line parameters
line_start = 10
line_offset = 20
line_color = (0, 255, 0)
line_thickness = 1

#------------------------------------------------------------------------------

if (__name__ == '__main__'):
    # Keyboard events ---------------------------------------------------------
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Get camera calibrations -------------------------------------------------
    calibration_lf = hl2ss_3dcv.get_calibration_rm(host, port_left, calibration_path)
    calibration_rf = hl2ss_3dcv.get_calibration_rm(host, port_right, calibration_path)

    rotation_lf = hl2ss_3dcv.rm_vlc_get_rotation(port_left)
    rotation_rf = hl2ss_3dcv.rm_vlc_get_rotation(port_right)

    K1, Rt1 = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_lf.intrinsics, calibration_lf.extrinsics, rotation_lf)
    K2, Rt2 = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_rf.intrinsics, calibration_rf.extrinsics, rotation_rf)

    # Get stereo calibration and rectify --------------------------------------
    stereo_calibration = hl2ss_3dcv.rm_vlc_stereo_calibrate(K1, K2, Rt1, Rt2)
    stereo_rectification = hl2ss_3dcv.rm_vlc_stereo_rectify(K1, K2, stereo_calibration.R, stereo_calibration.t, shape)

    # You can save stereo calibration and rectification using:
    # hl2ss_3dcv.save_stereo_calibration(port_left, port_right, stereo_calibration, calibration_path)
    # hl2ss_3dcv.save_stereo_rectification(port_left, port_right, stereo_rectification, calibration_path)
    # and load using:
    # stereo_calibration = hl2ss_3dcv.load_stereo_calibration(port_left, port_right, calibration_path)
    # stereo_rectification = hl2ss_3dcv.load_stereo_rectification(port_left, port_right, calibration_path)

    # Start streams -----------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(port_left, hl2ss_lnm.rx_rm_vlc(host, port_left))
    producer.configure(port_right, hl2ss_lnm.rx_rm_vlc(host, port_right))
    producer.initialize(port_left, buffer_size * hl2ss.Parameters_RM_VLC.FPS)
    producer.initialize(port_right, buffer_size * hl2ss.Parameters_RM_VLC.FPS)
    producer.start(port_left)
    producer.start(port_right)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_left = consumer.create_sink(producer, port_left, manager, ...)
    sink_right = consumer.create_sink(producer, port_right, manager, None)
    sink_left.get_attach_response()
    sink_right.get_attach_response()

    # Main loop ---------------------------------------------------------------
    while (enable):
        sink_left.acquire()

        # Get frames ----------------------------------------------------------
        _, _, data_left = sink_left.get_buffered_frame(-6)
        if (data_left is None):
            continue

        _, data_right = sink_right.get_nearest(data_left.timestamp)
        if (data_right is None):
            continue

        # Undistort and rectify frames ----------------------------------------
        lf_u = cv2.remap(data_left.payload.image, calibration_lf.undistort_map[:, :, 0], calibration_lf.undistort_map[:, :, 1], cv2.INTER_LINEAR)
        rf_u = cv2.remap(data_right.payload.image, calibration_rf.undistort_map[:, :, 0], calibration_rf.undistort_map[:, :, 1], cv2.INTER_LINEAR)

        lf_ru = hl2ss_3dcv.rm_vlc_rotate_image(lf_u, rotation_lf)
        rf_ru = hl2ss_3dcv.rm_vlc_rotate_image(rf_u, rotation_rf)

        r1 = cv2.remap(lf_ru, stereo_rectification.map1[:, :, 0], stereo_rectification.map1[:, :, 1], cv2.INTER_LINEAR)
        r2 = cv2.remap(rf_ru, stereo_rectification.map2[:, :, 0], stereo_rectification.map2[:, :, 1], cv2.INTER_LINEAR)

        # Display frames ------------------------------------------------------
        image_l = hl2ss_3dcv.rm_vlc_to_rgb(r1)
        image_r = hl2ss_3dcv.rm_vlc_to_rgb(r2)

        image = np.hstack((image_l, image_r))

        for y in range(line_start, shape[1], line_offset):
            cv2.line(image, (0, y), ((shape[0] * 2) - 1, y), line_color, line_thickness)
        
        cv2.imshow('Rectified', image)
        cv2.waitKey(1)

    # Stop streams ------------------------------------------------------------
    sink_left.detach()
    sink_right.detach()
    producer.stop(port_left)
    producer.stop(port_right)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
