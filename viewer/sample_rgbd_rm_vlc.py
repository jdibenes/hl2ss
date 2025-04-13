#------------------------------------------------------------------------------
# This script demonstrates how to create aligned "RGBD" images, which can be
# used with Open3D, from the depth and grayscale cameras of the HoloLens.
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

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# Port
vlc_port = hl2ss.StreamPort.RM_VLC_RIGHTFRONT

# Max depth in meters
max_depth = 3.0 

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()

    # Get RM VLC and RM Depth Long Throw calibration --------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration_vlc = hl2ss_3dcv.get_calibration_rm(calibration_path, host, vlc_port)
    calibration_lt = hl2ss_3dcv.get_calibration_rm(calibration_path, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)
    
    # Create Open3D visualizer ------------------------------------------------
    o3d_lt_intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    first_pcd = True

    # Start RM VLC and RM Depth Long Throw streams ----------------------------
    sink_vlc = hl2ss_mp.stream(hl2ss_lnm.rx_rm_vlc(host, vlc_port))
    sink_depth = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))

    sink_vlc.open()
    sink_depth.open()

    cv2.namedWindow('RGBD')

    # Main Loop ---------------------------------------------------------------
    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        cv2.waitKey(1)

        # Get RM Depth Long Throw frame and nearest (in time) RM VLC frame ----
        _, data_depth = sink_depth.get_most_recent_frame()
        if ((data_depth is None) or (not hl2ss.is_valid_pose(data_depth.pose))):
            continue

        _, data_vlc = sink_vlc.get_nearest(data_depth.timestamp)
        if ((data_vlc is None) or (not hl2ss.is_valid_pose(data_vlc.pose))):
            continue

        # Preprocess frames ---------------------------------------------------
        depth = hl2ss_3dcv.rm_depth_undistort(data_depth.payload.depth, calibration_lt.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = hl2ss_3dcv.rm_vlc_undistort(data_vlc.payload.image, calibration_vlc.undistort_map)

        # Generate aligned RGBD image -----------------------------------------
        lt_points          = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world        = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_depth.pose)
        world_to_lt        = hl2ss_3dcv.world_to_reference(data_depth.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
        world_to_vlc_image = hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_vlc.extrinsics) @ hl2ss_3dcv.camera_to_image(calibration_vlc.intrinsics)
        world_points       = hl2ss_3dcv.transform(lt_points, lt_to_world)
        vlc_uv             = hl2ss_3dcv.project(world_points, world_to_vlc_image)
        color              = cv2.remap(color, vlc_uv[:, :, 0], vlc_uv[:, :, 1], cv2.INTER_LINEAR)
        color              = hl2ss_3dcv.slice_to_block(color)

        mask_uv = hl2ss_3dcv.slice_to_block((vlc_uv[:, :, 0] < 0) | (vlc_uv[:, :, 0] >= hl2ss.Parameters_RM_VLC.WIDTH) | (vlc_uv[:, :, 1] < 0) | (vlc_uv[:, :, 1] >= hl2ss.Parameters_RM_VLC.HEIGHT))
        depth[mask_uv] = 0

        # Display RGBD --------------------------------------------------------
        color = hl2ss_3dcv.rm_vlc_to_rgb(color)
        
        image = np.hstack((hl2ss_3dcv.rm_depth_colormap(depth, max_depth), color))

        cv2.imshow('RGBD', image)

        # Convert to Open3D RGBD image ----------------------------------------
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

    # Stop RM VLC and RM Depth Long Throw streams -----------------------------
    sink_vlc.close()
    sink_depth.close()

    # Stop keyboard events ----------------------------------------------------
    listener.close()
