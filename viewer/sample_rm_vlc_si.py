#------------------------------------------------------------------------------
# This script receives video frames and spatial input data from the HoloLens.
# The received head pointer, hand joint positions and gaze pointer are 
# projected onto the video frame.
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

# Marker properties
radius = 5
head_color  = (  0,   0, 255)
left_color  = (  0, 255,   0)
right_color = (255,   0,   0)
gaze_color  = (255,   0, 255)
thickness = -1

# Spatial Mapping settings
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

    # Start RM VLC and Spatial Input streams ----------------------------------
    sink_vlc = hl2ss_mp.stream(hl2ss_lnm.rx_rm_vlc(host, vlc_port))
    sink_si = hl2ss_mp.stream(hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT))

    sink_vlc.open()
    sink_si.open()

    cv2.namedWindow('Video')

    # Main Loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Get RM VLC frame and nearest (in time) Spatial Input frame ----------
        _, data_vlc = sink_vlc.get_most_recent_frame()
        if (data_vlc is None):
            continue

        image = hl2ss_3dcv.rm_vlc_undistort(data_vlc.payload.image, calibration_vlc.undistort_map)
        image = hl2ss_3dcv.rm_vlc_to_rgb(image)

        if (not hl2ss.is_valid_pose(data_vlc.pose)):
            cv2.imshow('Video', hl2ss_3dcv.rm_vlc_rotate_image(image, rotation_vlc))
            continue

        _, data_si = sink_si.get_nearest(data_vlc.timestamp)
        if (data_si is None):
            cv2.imshow('Video', hl2ss_3dcv.rm_vlc_rotate_image(image, rotation_vlc))
            continue

        si = data_si.payload

        # Compute world to RM VLC image transformation matrix -----------------
        world_to_image = hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_vlc.extrinsics) @ hl2ss_3dcv.camera_to_image(calibration_vlc.intrinsics)

        # Draw Head Pointer ---------------------------------------------------
        if (si.head_pose_valid):
            head_pose = si.head_pose
            head_ray = hl2ss_3dcv.si_ray_to_vector(head_pose.position, head_pose.forward)
            d = sm_manager.cast_rays(head_ray)
            if (np.isfinite(d)):
                head_point = hl2ss_3dcv.si_ray_to_point(head_ray, d)
                head_image_point = hl2ss_3dcv.project(head_point, world_to_image)
                hl2ss_utilities.draw_points(image, head_image_point.astype(np.int32), radius, head_color, thickness)

        # Draw Left Hand joints -----------------------------------------------
        if (si.hand_left_valid):
            left_hand = si.hand_left
            left_image_points = hl2ss_3dcv.project(left_hand.position, world_to_image)
            hl2ss_utilities.draw_points(image, left_image_points.astype(np.int32), radius, left_color, thickness)

        # Draw Right Hand joints ----------------------------------------------
        if (si.hand_right_valid):
            right_hand = si.hand_right
            right_image_points = hl2ss_3dcv.project(right_hand.position, world_to_image)
            hl2ss_utilities.draw_points(image, right_image_points.astype(np.int32), radius, right_color, thickness)

        # Draw Gaze Pointer ---------------------------------------------------
        if (si.eye_ray_valid):
            eye_ray = si.eye_ray
            eye_ray_vector = hl2ss_3dcv.si_ray_to_vector(eye_ray.origin, eye_ray.direction)
            d = sm_manager.cast_rays(eye_ray_vector)
            if (np.isfinite(d)):
                gaze_point = hl2ss_3dcv.si_ray_to_point(eye_ray_vector, d)
                gaze_image_point = hl2ss_3dcv.project(gaze_point, world_to_image)
                hl2ss_utilities.draw_points(image, gaze_image_point.astype(np.int32), radius, gaze_color, thickness)
                
        # Display frame -------------------------------------------------------
        cv2.imshow('Video', hl2ss_3dcv.rm_vlc_rotate_image(image, rotation_vlc))

    sink_vlc.close()
    sink_si.close()
