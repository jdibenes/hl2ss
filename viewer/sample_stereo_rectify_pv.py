#------------------------------------------------------------------------------
# This script performs stereo rectification for the RM VLC left front camera
# and PV. Before running this script, extract the PV rignode extrinsics using
# tools/pv_extrinsic_calibration.py.
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

# Calibration folder (must exist but can be empty)
calibration_path = '../calibration'

# Camera parameters
pv_focus = 1000 # in mm
pv_width = 640
pv_height = 360
pv_framerate = 30

# Buffer size in seconds
buffer_size = 10

# Undistort rectify map shape
shape = (640,640)

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

    # Start PV subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Fix PV camera focus so intrinsics do not change between frames ----------
    ipc_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    ipc_rc.open()
    ipc_rc.wait_for_pv_subsystem(True)
    ipc_rc.set_pv_focus(hl2ss.PV_FocusMode.Manual, hl2ss.PV_AutoFocusRange.Normal, hl2ss.PV_ManualFocusDistance.Infinity, pv_focus, hl2ss.PV_DriverFallback.Disable)
    ipc_rc.close()

    # Get camera calibrations -------------------------------------------------
    port_left  = hl2ss.StreamPort.RM_VLC_LEFTFRONT

    calibration_lf = hl2ss_3dcv.get_calibration_rm(host, port_left, calibration_path)
    calibration_rf = hl2ss_3dcv.get_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, calibration_path, pv_focus, pv_width, pv_height, pv_framerate)

    rotation_lf = hl2ss_3dcv.rm_vlc_get_rotation(port_left)

    K1, Rt1 = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_lf.intrinsics, calibration_lf.extrinsics, rotation_lf)
    K2, Rt2 = hl2ss_3dcv.pv_fix_calibration(calibration_rf.intrinsics, calibration_rf.extrinsics)

    # Get stereo calibration and rectify --------------------------------------
    stereo_calibration   = hl2ss_3dcv.rm_vlc_stereo_calibrate(K1, K2, Rt1, Rt2)
    stereo_rectification = hl2ss_3dcv.rm_vlc_stereo_rectify(K1, K2, stereo_calibration.R, stereo_calibration.t, shape)

    # Start streams -----------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(port_left, hl2ss_lnm.rx_rm_vlc(host, port_left))
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))
    producer.initialize(port_left, buffer_size * hl2ss.Parameters_RM_VLC.FPS)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, buffer_size * pv_framerate)
    producer.start(port_left)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_left = consumer.create_sink(producer, port_left, manager, ...)
    sink_right = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_left.get_attach_response()
    sink_right.get_attach_response()

    # Main loop ---------------------------------------------------------------
    while (enable):
        sink_left.acquire()

        # Get frames ----------------------------------------------------------
        _, data_left = sink_left.get_most_recent_frame()
        if (data_left is None):
            continue

        _, data_right = sink_right.get_nearest(data_left.timestamp)
        if (data_right is None):
            continue

        # Undistort and rectify frames ----------------------------------------
        lf_u = cv2.remap(data_left.payload.image, calibration_lf.undistort_map[:, :, 0], calibration_lf.undistort_map[:, :, 1], cv2.INTER_LINEAR)
        lf_ru = hl2ss_3dcv.rm_vlc_rotate_image(lf_u, rotation_lf)
        rf_ru = data_right.payload.image

        r1 = cv2.remap(lf_ru, stereo_rectification.map1[:, :, 0], stereo_rectification.map1[:, :, 1], cv2.INTER_LINEAR)
        r2 = cv2.remap(rf_ru, stereo_rectification.map2[:, :, 0], stereo_rectification.map2[:, :, 1], cv2.INTER_LINEAR)

        # Display frames ------------------------------------------------------
        image_l = hl2ss_3dcv.rm_vlc_to_rgb(r1)
        image_r = r2

        image = np.hstack((image_l, image_r))

        for y in range(line_start, shape[1], line_offset):
            cv2.line(image, (0, y), ((shape[0] * 2) - 1, y), line_color, line_thickness)
        
        cv2.imshow('Rectified', image)
        cv2.waitKey(1)

    # Stop streams ------------------------------------------------------------
    sink_left.detach()
    sink_right.detach()
    producer.stop(port_left)
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
