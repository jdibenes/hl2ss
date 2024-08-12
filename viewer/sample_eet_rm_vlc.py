#------------------------------------------------------------------------------
# This script receives video frames and extended eye tracking data from the 
# HoloLens. The received left, right, and combined gaze pointers are projected
# onto the video frame.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
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
eet_fps = 30 # 30, 60, 90

# Marker properties
radius = 5
combined_color = (255, 0, 255)
left_color     = (  0, 0, 255)
right_color    = (255, 0,   0)
thickness = -1

# Buffer length in seconds
buffer_length = 5

# Spatial Mapping manager settings
triangles_per_cubic_meter = 1000
mesh_threads = 2
sphere_center = [0, 0, 0]
sphere_radius = 5

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Start Spatial Mapping data manager --------------------------------------
    # Set region of 3D space to sample
    volumes = hl2ss.sm_bounding_volume()
    volumes.add_sphere(sphere_center, sphere_radius)

    # Download observed surfaces
    sm_manager = hl2ss_sa.sm_manager(host, triangles_per_cubic_meter, mesh_threads)
    sm_manager.open()
    sm_manager.set_volumes(volumes)
    sm_manager.get_observed_surfaces()

    # Get RM VLC calibration --------------------------------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration_vlc = hl2ss_3dcv.get_calibration_rm(host, vlc_port, calibration_path)
    rotation_vlc = hl2ss_3dcv.rm_vlc_get_rotation(vlc_port)
    
    # Start RM VLC and EET streams --------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(vlc_port, hl2ss_lnm.rx_rm_vlc(host, vlc_port))
    producer.configure(hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss_lnm.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, fps=eet_fps))
    producer.initialize(vlc_port, hl2ss.Parameters_RM_VLC.FPS * buffer_length)
    producer.initialize(hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(vlc_port)
    producer.start(hl2ss.StreamPort.EXTENDED_EYE_TRACKER)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_vlc = consumer.create_sink(producer, vlc_port, manager, ...)
    sink_eet = consumer.create_sink(producer, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, manager, None)
    sink_vlc.get_attach_response()
    sink_eet.get_attach_response()

    # Main Loop ---------------------------------------------------------------
    while (enable):
        # Download observed surfaces ------------------------------------------
        sm_manager.get_observed_surfaces()

        # Wait for RM VLC frame -----------------------------------------------
        sink_vlc.acquire()

        # Get RM VLC frame and nearest (in time) EET frame --------------------
        _, data_vlc = sink_vlc.get_most_recent_frame()
        if ((data_vlc is None) or (not hl2ss.is_valid_pose(data_vlc.pose))):
            continue

        _, data_eet = sink_eet.get_nearest(data_vlc.timestamp)
        if ((data_eet is None) or (not hl2ss.is_valid_pose(data_eet.pose))):
            continue

        image = cv2.remap(data_vlc.payload.image, calibration_vlc.undistort_map[:, :, 0], calibration_vlc.undistort_map[:, :, 1], cv2.INTER_LINEAR)
        image = np.dstack((image, image, image))
        eet = hl2ss.unpack_eet(data_eet.payload)

        # Compute world to RM VLC image transformation matrix -----------------
        world_to_image = hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_vlc.extrinsics) @ hl2ss_3dcv.camera_to_image(calibration_vlc.intrinsics)

        # Draw Left Gaze Pointer ----------------------------------------------
        if (eet.left_ray_valid):
            local_left_ray = hl2ss_utilities.si_ray_to_vector(eet.left_ray.origin, eet.left_ray.direction)
            left_ray = hl2ss_utilities.si_ray_transform(local_left_ray, data_eet.pose)
            d = sm_manager.cast_rays(left_ray)
            if (np.isfinite(d)):
                left_point = hl2ss_utilities.si_ray_to_point(left_ray, d)
                left_image_point = hl2ss_3dcv.project(left_point, world_to_image)
                hl2ss_utilities.draw_points(image, left_image_point.astype(np.int32), radius, left_color, thickness)

        # Draw Right Gaze Pointer ---------------------------------------------
        if (eet.right_ray_valid):
            local_right_ray = hl2ss_utilities.si_ray_to_vector(eet.right_ray.origin, eet.right_ray.direction)
            right_ray = hl2ss_utilities.si_ray_transform(local_right_ray, data_eet.pose)
            d = sm_manager.cast_rays(right_ray)
            if (np.isfinite(d)):
                right_point = hl2ss_utilities.si_ray_to_point(right_ray, d)
                right_image_point = hl2ss_3dcv.project(right_point, world_to_image)
                hl2ss_utilities.draw_points(image, right_image_point.astype(np.int32), radius, right_color, thickness)

        # Draw Combined Gaze Pointer ------------------------------------------
        if (eet.combined_ray_valid):
            local_combined_ray = hl2ss_utilities.si_ray_to_vector(eet.combined_ray.origin, eet.combined_ray.direction)
            combined_ray = hl2ss_utilities.si_ray_transform(local_combined_ray, data_eet.pose)
            d = sm_manager.cast_rays(combined_ray)
            if (np.isfinite(d)):
                combined_point = hl2ss_utilities.si_ray_to_point(combined_ray, d)
                combined_image_point = hl2ss_3dcv.project(combined_point, world_to_image)
                hl2ss_utilities.draw_points(image, combined_image_point.astype(np.int32), radius, combined_color, thickness)

        # Display frame -------------------------------------------------------            
        cv2.imshow('Video', hl2ss_3dcv.rm_vlc_rotate_image(image, rotation_vlc))
        cv2.waitKey(1)

    # Stop Spatial Mapping data manager ---------------------------------------
    sm_manager.close()

    # Stop RM VLC and EET streams ---------------------------------------------
    sink_vlc.detach()
    sink_eet.detach()
    producer.stop(vlc_port)
    producer.stop(hl2ss.StreamPort.EXTENDED_EYE_TRACKER)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
