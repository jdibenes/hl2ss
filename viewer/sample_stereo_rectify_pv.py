#------------------------------------------------------------------------------
# This script performs stereo rectification for the RM VLC left front camera
# and PV. Before running this script, extract the PV rignode extrinsics using
# tools/pv_extrinsic_calibration.py.
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

# Calibration folder (must exist but can be empty)
calibration_path = '../calibration'

# Camera parameters
pv_focus = 1000 # in mm
pv_width = 640
pv_height = 360
pv_framerate = 30

# Undistort rectify map shape
shape = (640,640)

# Line parameters
line_start = 10
line_offset = 20
line_color = (0, 255, 0)
line_thickness = 1

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Start PV subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Fix PV camera focus so intrinsics do not change between frames ----------
    ipc_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    ipc_rc.open()
    ipc_rc.pv_wait_for_subsystem(True)
    ipc_rc.pv_set_focus(hl2ss.PV_FocusMode.Manual, hl2ss.PV_AutoFocusRange.Normal, hl2ss.PV_ManualFocusDistance.Infinity, pv_focus, hl2ss.PV_DriverFallback.Disable)
    ipc_rc.close()

    # Get camera calibrations -------------------------------------------------
    port_left  = hl2ss.StreamPort.RM_VLC_LEFTFRONT

    calibration_lf = hl2ss_3dcv.get_calibration_rm(calibration_path, host, port_left)
    calibration_rf = hl2ss_3dcv.get_calibration_pv(calibration_path, host, hl2ss.StreamPort.PERSONAL_VIDEO, focus=pv_focus, width=pv_width, height=pv_height, framerate=pv_framerate)

    rotation_lf = hl2ss_3dcv.rm_vlc_get_rotation(port_left)

    K1, Rt1 = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_lf.intrinsics, calibration_lf.extrinsics, rotation_lf)
    K2, Rt2 = hl2ss_3dcv.pv_fix_calibration(calibration_rf.intrinsics, calibration_rf.extrinsics)

    # Get stereo calibration and rectify --------------------------------------
    stereo_calibration   = hl2ss_3dcv.rm_vlc_stereo_calibrate(K1, K2, Rt1, Rt2)
    stereo_rectification = hl2ss_3dcv.rm_vlc_stereo_rectify(K1, K2, stereo_calibration.R, stereo_calibration.t, shape)

    # Start streams -----------------------------------------------------------
    sink_left = hl2ss_mp.stream(hl2ss_lnm.rx_rm_vlc(host, port_left))
    sink_right = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))

    sink_left.open()
    sink_right.open()

    cv2.namedWindow('Rectified')

    # Main loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Get frames ----------------------------------------------------------
        _, data_left = sink_left.get_most_recent_frame()
        if (data_left is None):
            continue

        _, data_right = sink_right.get_nearest(data_left.timestamp)
        if (data_right is None):
            continue

        # Undistort and rectify frames ----------------------------------------
        lf_u = hl2ss_3dcv.rm_vlc_undistort(data_left.payload.image, calibration_lf.undistort_map)
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

    # Stop streams ------------------------------------------------------------
    sink_left.close()
    sink_right.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
