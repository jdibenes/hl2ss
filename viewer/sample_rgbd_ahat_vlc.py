#------------------------------------------------------------------------------
# This script demonstrates how to create aligned "RGBD" images, which can be
# used with Open3D, from the depth and grayscale cameras of the HoloLens.
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

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# Port
vlc_port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

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

    # Get RM VLC and RM Depth Long Throw calibration --------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration_vlc = hl2ss_3dcv.get_calibration_rm(host, vlc_port, calibration_path)
    calibration_ht = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_AHAT, calibration_path)

    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_ht.intrinsics, hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_ht.scale)
    max_depth = calibration_ht.alias / calibration_ht.scale

    # Create Open3D visualizer ------------------------------------------------
    o3d_lt_intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT, calibration_ht.intrinsics[0, 0], calibration_ht.intrinsics[1, 1], calibration_ht.intrinsics[2, 0], calibration_ht.intrinsics[2, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    first_pcd = True

    # Start RM VLC and RM Depth AHAT streams ----------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(vlc_port, hl2ss_lnm.rx_rm_vlc(host, vlc_port))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT, profile_z=ht_profile_z, profile_ab=ht_profile_ab))
    producer.initialize(vlc_port, buffer_length * hl2ss.Parameters_RM_VLC.FPS)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_AHAT, buffer_length * hl2ss.Parameters_RM_DEPTH_AHAT.FPS)
    producer.start(vlc_port)
    producer.start(hl2ss.StreamPort.RM_DEPTH_AHAT)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_vlc = consumer.create_sink(producer, vlc_port, manager, None)
    sink_ht = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_AHAT, manager, ...)
    
    sink_vlc.get_attach_response()
    sink_ht.get_attach_response()

    # Main Loop ---------------------------------------------------------------
    while (enable):
        # Wait for RM Depth AHAT frame ----------------------------------------
        sink_ht.acquire()

        # Get RM Depth AHAT frame and nearest (in time) RM VLC frame ----------
        _, data_ht = sink_ht.get_most_recent_frame()
        if ((data_ht is None) or (not hl2ss.is_valid_pose(data_ht.pose))):
            continue

        _, data_vlc = sink_vlc.get_nearest(data_ht.timestamp)
        if ((data_vlc is None) or (not hl2ss.is_valid_pose(data_vlc.pose))):
            continue

        # Preprocess frames ---------------------------------------------------
        depth = hl2ss_3dcv.rm_depth_undistort(data_ht.payload.depth, calibration_ht.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = cv2.remap(data_vlc.payload.image, calibration_vlc.undistort_map[:, :, 0], calibration_vlc.undistort_map[:, :, 1], cv2.INTER_LINEAR)

        # Generate aligned RGBD image -----------------------------------------
        lt_points          = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world        = hl2ss_3dcv.camera_to_rignode(calibration_ht.extrinsics) @ hl2ss_3dcv.reference_to_world(data_ht.pose)
        world_to_lt        = hl2ss_3dcv.world_to_reference(data_ht.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_ht.extrinsics)
        world_to_vlc_image = hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_vlc.extrinsics) @ hl2ss_3dcv.camera_to_image(calibration_vlc.intrinsics)
        world_points       = hl2ss_3dcv.transform(lt_points, lt_to_world)
        vlc_uv             = hl2ss_3dcv.project(world_points, world_to_vlc_image)
        color              = cv2.remap(color, vlc_uv[:, :, 0], vlc_uv[:, :, 1], cv2.INTER_LINEAR)
        color              = hl2ss_3dcv.slice_to_block(color)

        mask_uv = hl2ss_3dcv.slice_to_block((vlc_uv[:, :, 0] < 0) | (vlc_uv[:, :, 0] >= hl2ss.Parameters_RM_VLC.WIDTH) | (vlc_uv[:, :, 1] < 0) | (vlc_uv[:, :, 1] >= hl2ss.Parameters_RM_VLC.HEIGHT))
        depth[mask_uv] = 0

        # Display RGBD --------------------------------------------------------
        image = np.hstack((depth / max_depth, color / 255)) # Depth scaled for visibility
        cv2.imshow('RGBD', image)
        cv2.waitKey(1)

        # Convert to Open3D RGBD image ----------------------------------------
        color = hl2ss_3dcv.rm_vlc_to_rgb(color)

        color_image = o3d.geometry.Image(color)
        depth_image = o3d.geometry.Image(depth)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)

        tmp_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_lt_intrinsics)

        # Display pointcloud --------------------------------------------------
        pcd.points = tmp_pcd.points
        pcd.colors = tmp_pcd.colors

        if (first_pcd):
            vis.add_geometry(pcd)
            first_pcd = False
        else:
            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()

    # Stop RM VLC and RM Depth AHAT streams -----------------------------------
    sink_vlc.detach()
    sink_ht.detach()
    producer.stop(vlc_port)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_AHAT)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
