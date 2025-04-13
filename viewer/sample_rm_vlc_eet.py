#------------------------------------------------------------------------------
# This script receives video frames and extended eye tracking data from the 
# HoloLens. The received left, right, and combined gaze pointers are projected
# onto the video frame.
# Press esc to stop.
#------------------------------------------------------------------------------

import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_utilities
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = "192.168.1.7"

# Calibration folder (must exist but can be empty)
calibration_path = '../calibration'

# Port
vlc_port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

# EET parameters
eet_fps = 90 # 30, 60, 90

# Marker properties
radius = 5
combined_color = (255, 0, 255)
left_color     = (  0, 0, 255)
right_color    = (255, 0,   0)
thickness = -1

# Spatial Mapping manager settings
triangles_per_cubic_meter = 1000
sphere_center = [0, 0, 0]
sphere_radius = 5

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Start Spatial Mapping data manager --------------------------------------
    # Set region of 3D space to sample
    volumes = hl2ss.sm_bounding_volume()
    volumes.add_sphere(sphere_center, sphere_radius)

    # Download observed surfaces
    sm_manager = hl2ss_sa.sm_manager(host, hl2ss.IPCPort.SPATIAL_MAPPING, triangles_per_cubic_meter=triangles_per_cubic_meter)
    sm_manager.open()
    sm_manager.set_volumes(volumes)
    print('downloading sm meshes...')
    sm_manager.get_observed_surfaces()
    print('done')
    sm_manager.close()

    # Get RM VLC calibration --------------------------------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration_vlc = hl2ss_3dcv.get_calibration_rm(calibration_path, host, vlc_port)
    rotation_vlc = hl2ss_3dcv.rm_vlc_get_rotation(vlc_port)

    # Start RM VLC and EET streams --------------------------------------------
    sink_vlc = hl2ss_mp.stream(hl2ss_lnm.rx_rm_vlc(host, vlc_port))
    sink_eet = hl2ss_mp.stream(hl2ss_lnm.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, fps=eet_fps))

    sink_vlc.open()
    sink_eet.open()

    cv2.namedWindow('Video')

    # Main Loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Get RM VLC frame and nearest (in time) EET frame --------------------
        _, data_vlc = sink_vlc.get_most_recent_frame()
        if (data_vlc is None):
            continue

        image = hl2ss_3dcv.rm_vlc_undistort(data_vlc.payload.image, calibration_vlc.undistort_map)
        image = hl2ss_3dcv.rm_vlc_to_rgb(image)

        if (not hl2ss.is_valid_pose(data_vlc.pose)):
            cv2.imshow('Video', hl2ss_3dcv.rm_vlc_rotate_image(image, rotation_vlc))
            continue

        _, data_eet = sink_eet.get_nearest(data_vlc.timestamp)
        if ((data_eet is None) or (not hl2ss.is_valid_pose(data_eet.pose))):
            cv2.imshow('Video', hl2ss_3dcv.rm_vlc_rotate_image(image, rotation_vlc))
            continue

        eet = data_eet.payload

        # Compute world to RM VLC image transformation matrix -----------------
        world_to_image = hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_vlc.extrinsics) @ hl2ss_3dcv.camera_to_image(calibration_vlc.intrinsics)

        # Draw Left Gaze Pointer ----------------------------------------------
        if (eet.left_ray_valid):
            local_left_ray = hl2ss_3dcv.si_ray_to_vector(eet.left_ray.origin, eet.left_ray.direction)
            left_ray = hl2ss_3dcv.si_ray_transform(local_left_ray, data_eet.pose)
            d = sm_manager.cast_rays(left_ray)
            if (np.isfinite(d)):
                left_point = hl2ss_3dcv.si_ray_to_point(left_ray, d)
                left_image_point = hl2ss_3dcv.project(left_point, world_to_image)
                hl2ss_utilities.draw_points(image, left_image_point.astype(np.int32), radius, left_color, thickness)

        # Draw Right Gaze Pointer ---------------------------------------------
        if (eet.right_ray_valid):
            local_right_ray = hl2ss_3dcv.si_ray_to_vector(eet.right_ray.origin, eet.right_ray.direction)
            right_ray = hl2ss_3dcv.si_ray_transform(local_right_ray, data_eet.pose)
            d = sm_manager.cast_rays(right_ray)
            if (np.isfinite(d)):
                right_point = hl2ss_3dcv.si_ray_to_point(right_ray, d)
                right_image_point = hl2ss_3dcv.project(right_point, world_to_image)
                hl2ss_utilities.draw_points(image, right_image_point.astype(np.int32), radius, right_color, thickness)

        # Draw Combined Gaze Pointer ------------------------------------------
        if (eet.combined_ray_valid):
            local_combined_ray = hl2ss_3dcv.si_ray_to_vector(eet.combined_ray.origin, eet.combined_ray.direction)
            combined_ray = hl2ss_3dcv.si_ray_transform(local_combined_ray, data_eet.pose)
            d = sm_manager.cast_rays(combined_ray)
            if (np.isfinite(d)):
                combined_point = hl2ss_3dcv.si_ray_to_point(combined_ray, d)
                combined_image_point = hl2ss_3dcv.project(combined_point, world_to_image)
                hl2ss_utilities.draw_points(image, combined_image_point.astype(np.int32), radius, combined_color, thickness)

        # Display frame -------------------------------------------------------            
        cv2.imshow('Video', hl2ss_3dcv.rm_vlc_rotate_image(image, rotation_vlc))

    # Stop RM VLC and EET streams ---------------------------------------------
    sink_vlc.close()
    sink_eet.close()
