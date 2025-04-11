#------------------------------------------------------------------------------
# This script performs stereo rectification for a pair of RM VLC cameras.
# Press esc to stop.
#------------------------------------------------------------------------------

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

# Undistort rectify map shape
shape = hl2ss.Parameters_RM_VLC.SHAPE

# Line parameters
line_start = 10
line_offset = 20
line_color = (0, 255, 0)
line_thickness = 1

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Get camera calibrations -------------------------------------------------
    calibration_lf = hl2ss_3dcv.get_calibration_rm(calibration_path, host, port_left)
    calibration_rf = hl2ss_3dcv.get_calibration_rm(calibration_path, host, port_right)

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
    sink_left = hl2ss_mp.stream(hl2ss_lnm.rx_rm_vlc(host, port_left))
    sink_right = hl2ss_mp.stream(hl2ss_lnm.rx_rm_vlc(host, port_right))

    sink_left.open()
    sink_right.open()

    cv2.namedWindow('Rectified')

    # Main loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Get frames ----------------------------------------------------------
        _, _, data_left = sink_left.get_buffered_frame(-6)
        if (data_left is None):
            continue

        _, data_right = sink_right.get_nearest(data_left.timestamp)
        if (data_right is None):
            continue

        # Undistort and rectify frames ----------------------------------------
        lf_u = hl2ss_3dcv.rm_vlc_undistort(data_left.payload.image, calibration_lf.undistort_map)
        rf_u = hl2ss_3dcv.rm_vlc_undistort(data_right.payload.image, calibration_rf.undistort_map)

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

    # Stop streams ------------------------------------------------------------
    sink_left.close()
    sink_right.close()
