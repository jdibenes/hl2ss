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
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = "192.168.1.7"

# Camera parameters
pv_width     = 760
pv_height    = 428
pv_framerate = 30

# EET parameters
# 30, 60, 90
eet_fps = 90 

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
    # Start PV Subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Start Spatial Mapping data manager --------------------------------------
    # SM meshes are used to determine the depth of the eye gaze rays via 
    # raycasting
    volumes = hl2ss.sm_bounding_volume()
    volumes.add_sphere(sphere_center, sphere_radius)

    sm_manager = hl2ss_sa.sm_manager_mp(host, hl2ss.IPCPort.SPATIAL_MAPPING, triangles_per_cubic_meter=triangles_per_cubic_meter)
    sm_manager.open()
    sm_manager.set_volumes(volumes)
    sm_manager.get_observed_surfaces()
    
    # Start PV and EET streams ------------------------------------------------
    sink_pv  = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))
    sink_eet = hl2ss_mp.stream(hl2ss_lnm.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, fps=eet_fps))
    
    sink_pv.open()
    sink_eet.open()

    cv2.namedWindow('Video')

    # Main Loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Update observed surfaces --------------------------------------------
        sm_manager.get_observed_surfaces()

        # Get PV frame and nearest (in time) EET frame ------------------------
        _, data_pv = sink_pv.get_most_recent_frame()
        if (data_pv is None):
            continue

        image = data_pv.payload.image

        if (not hl2ss.is_valid_pose(data_pv.pose)):
            cv2.imshow('Video', image)
            continue

        _, data_eet = sink_eet.get_nearest(data_pv.timestamp)
        if ((data_eet is None) or (not hl2ss.is_valid_pose(data_eet.pose))):
            cv2.imshow('Video', image)
            continue

        eet = data_eet.payload

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss_3dcv.pv_create_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4, dtype=np.float32)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        # Compute world to PV image transformation matrix ---------------------
        world_to_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(pv_extrinsics) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)

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
        cv2.imshow('Video', image)
    
    # Stop PV and EET streams -------------------------------------------------
    sink_pv.close()
    sink_eet.close()

    # Stop Spatial Mapping data manager ---------------------------------------
    sm_manager.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
