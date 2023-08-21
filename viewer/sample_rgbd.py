#------------------------------------------------------------------------------
# This script receives depth frames from the HoloLens and converts them to 
# pointclouds.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import open3d as o3d
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Port: RM Depth AHAT or RM Depth Long Throw
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# Use AB data to color the pointcloud
use_ab = False

# AHAT Profile
ht_profile_z = hl2ss.DepthProfile.SAME
ht_profile_ab = hl2ss.VideoProfile.H265_MAIN

# Buffer length in seconds
buffer_length = 10

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Get calibration ---------------------------------------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration = hl2ss_3dcv.get_calibration_rm(host, port, calibration_path)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(calibration.uv2xy, calibration.scale)
    max_depth = 8.0 if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW) else (calibration.alias / calibration.scale)
    fps = hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW) else hl2ss.Parameters_RM_DEPTH_AHAT.FPS

    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    first_pcd = True

    # Start stream ------------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT, profile_z=ht_profile_z, profile_ab=ht_profile_ab))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    producer.initialize(port, buffer_length * fps)
    producer.start(port)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()    
    sink_depth = consumer.create_sink(producer, port, manager, ...)
    sink_depth.get_attach_response()

    # Main Loop ---------------------------------------------------------------
    while (enable):
        # Wait for frame ------------------------------------------------------
        sink_depth.acquire()

        # Get frame -----------------------------------------------------------
        _, data = sink_depth.get_most_recent_frame()
        if (data is None):
            continue

        depth = hl2ss_3dcv.rm_depth_normalize(data.payload.depth, scale)
        ab = hl2ss_3dcv.slice_to_block(data.payload.ab) / 65536

        # Display RGBD --------------------------------------------------------
        image = np.hstack((depth / max_depth, ab)) # Depth scaled for visibility
        cv2.imshow('RGBD', image)
        cv2.waitKey(1)

        # Display pointcloud --------------------------------------------------
        xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
        xyz = hl2ss_3dcv.block_to_list(xyz)
        rgb = hl2ss_3dcv.block_to_list(ab)
        d = hl2ss_3dcv.block_to_list(depth).reshape((-1,))
        xyz = xyz[d > 0, :]
        rgb = rgb[d > 0, :]
        rgb = np.hstack((rgb, rgb, rgb))

        pcd.points = o3d.utility.Vector3dVector(xyz)
        if (use_ab):
            pcd.colors = o3d.utility.Vector3dVector(rgb)

        if (first_pcd):
            vis.add_geometry(pcd)
            first_pcd = False
        else:
            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()

    # Stop stream -------------------------------------------------------------
    sink_depth.detach()
    producer.stop(port)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
