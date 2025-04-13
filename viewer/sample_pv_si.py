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

# Spatial Input sampling delay
# Delays SI readouts by the specified time (in hundreds of nanoseconds)
# Negative values (future timestamps) enable HoloLens predictions
sampling_delay = hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS // hl2ss.Parameters_SI.SAMPLE_RATE

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Set SI sampling delay
    client_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    client_rc.open()
    client_rc.si_set_sampling_delay(sampling_delay)
    client_rc.close()

    # Start PV Subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Start Spatial Mapping data manager --------------------------------------
    # SM meshes are used to determine the depth of the head pointer and eye 
    # gaze rays via raycasting
    volumes = hl2ss.sm_bounding_volume()
    volumes.add_sphere(sphere_center, sphere_radius)

    sm_manager = hl2ss_sa.sm_manager_mp(host, hl2ss.IPCPort.SPATIAL_MAPPING, triangles_per_cubic_meter=triangles_per_cubic_meter)
    sm_manager.open()
    sm_manager.set_volumes(volumes)
    sm_manager.get_observed_surfaces()
    
    # Start PV and Spatial Input streams --------------------------------------
    sink_pv = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))
    sink_si = hl2ss_mp.stream(hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT))

    sink_pv.open()
    sink_si.open()

    cv2.namedWindow('Video')

    # Main Loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Update observed surfaces --------------------------------------------
        sm_manager.get_observed_surfaces()

        # Get PV frame and nearest (in time) Spatial Input frame --------------
        _, _, data_pv = sink_pv.get_buffered_frame(-4)
        if (data_pv is None):
            continue

        image = data_pv.payload.image

        if (not hl2ss.is_valid_pose(data_pv.pose)):
            cv2.imshow('Video', image)
            continue

        _, data_si = sink_si.get_nearest(data_pv.timestamp)
        if (data_si is None):
            cv2.imshow('Video', image)
            continue

        si = data_si.payload

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss_3dcv.pv_create_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4, dtype=np.float32)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        # Compute world to PV image transformation matrix ---------------------
        world_to_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(pv_extrinsics) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)

        # Draw Head Pointer ---------------------------------------------------
        if (si.head_pose_valid):
            head_pose = si.head_pose
            head_ray = hl2ss_3dcv.si_ray_to_vector(head_pose.position, head_pose.forward)
            d = sm_manager.cast_rays(head_ray)
            if (np.isfinite(d)):
                head_point = hl2ss_3dcv.si_ray_to_point(head_ray, d)
                head_image_point = hl2ss_3dcv.project(head_point, world_to_image)
                hl2ss_utilities.draw_points(image, head_image_point.astype(np.int32), radius, head_color, thickness)

        # Draw Gaze Pointer ---------------------------------------------------
        if (si.eye_ray_valid):
            eye_ray = si.eye_ray
            eye_ray_vector = hl2ss_3dcv.si_ray_to_vector(eye_ray.origin, eye_ray.direction)
            d = sm_manager.cast_rays(eye_ray_vector)
            if (np.isfinite(d)):
                gaze_point = hl2ss_3dcv.si_ray_to_point(eye_ray_vector, d)
                gaze_image_point = hl2ss_3dcv.project(gaze_point, world_to_image)
                hl2ss_utilities.draw_points(image, gaze_image_point.astype(np.int32), radius, gaze_color, thickness)

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

        # Display frame -------------------------------------------------------
        cv2.imshow('Video', image)

    # Stop PV and Spatial Input streams ---------------------------------------
    sink_pv.close()
    sink_si.close()

    # Stop Spatial Mapping data manager ---------------------------------------
    sm_manager.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
