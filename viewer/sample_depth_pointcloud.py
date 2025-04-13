#------------------------------------------------------------------------------
# This script receives depth frames from the HoloLens and converts them to 
# pointclouds.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import open3d as o3d
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_utilities

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Port: RM Depth AHAT or RM Depth Long Throw
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# AHAT Profile
ht_profile_z = hl2ss.DepthProfile.SAME
ht_profile_ab = hl2ss.VideoProfile.H265_MAIN

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()

    # Get calibration ---------------------------------------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration = hl2ss_3dcv.get_calibration_rm(calibration_path, host, port)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(calibration.uv2xy, calibration.scale)
    max_depth = 7.5 if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW) else (calibration.alias / calibration.scale) if (port == hl2ss.StreamPort.RM_DEPTH_AHAT) else None

    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    first_pcd = True

    cv2.namedWindow('Depth')
    cv2.namedWindow('AB')

    # Start stream ------------------------------------------------------------
    rx_ht = hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT, profile_z=ht_profile_z, profile_ab=ht_profile_ab)
    rx_lt = hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    sink_depth = hl2ss_mp.stream(rx_lt if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW) else rx_ht if (port == hl2ss.StreamPort.RM_DEPTH_AHAT) else None)
    sink_depth.open()

    # Main Loop ---------------------------------------------------------------
    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        cv2.waitKey(1)

        # Get frame -----------------------------------------------------------
        _, data = sink_depth.get_most_recent_frame()
        if (data is None):
            continue

        depth = hl2ss_3dcv.rm_depth_normalize(data.payload.depth, scale)
        ab = hl2ss_3dcv.slice_to_block(hl2ss_3dcv.rm_ab_normalize(data.payload.ab)) # scaled for visibility

        # Display RGBD --------------------------------------------------------

        cv2.imshow('Depth', hl2ss_3dcv.rm_depth_colormap(depth, max_depth))
        cv2.imshow('AB', ab)
       
        # Display pointcloud --------------------------------------------------
        xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
        xyz = hl2ss_3dcv.block_to_list(xyz)
        rgb = hl2ss_3dcv.block_to_list(ab)
        d = hl2ss_3dcv.block_to_list(depth).reshape((-1,))
        xyz = xyz[d > 0, :]
        rgb = rgb[d > 0, :] / 255
        rgb = np.hstack((rgb, rgb, rgb))

        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(rgb)

        if (first_pcd):
            vis.add_geometry(pcd)
            first_pcd = False
        else:
            vis.update_geometry(pcd)

    # Stop stream -------------------------------------------------------------
    sink_depth.close()

    # Stop keyboard events ----------------------------------------------------
    listener.close()
