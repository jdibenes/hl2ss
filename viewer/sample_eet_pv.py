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

# Camera parameters
# See etc/hl2_capture_formats.txt for a list of supported formats
pv_width     = 760
pv_height    = 428
pv_framerate = 30

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

    # Start PV Subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Start Spatial Mapping data manager --------------------------------------
    # Set region of 3D space to sample
    volumes = hl2ss.sm_bounding_volume()
    volumes.add_sphere(sphere_center, sphere_radius)

    # Download observed surfaces
    sm_manager = hl2ss_sa.sm_manager(host, triangles_per_cubic_meter, mesh_threads)
    sm_manager.open()
    sm_manager.set_volumes(volumes)
    sm_manager.get_observed_surfaces()
    
    # Start PV and EET streams ------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))
    producer.configure(hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss_lnm.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, fps=eet_fps))
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, pv_framerate * buffer_length)
    producer.initialize(hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.EXTENDED_EYE_TRACKER)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, ...)
    sink_eet = consumer.create_sink(producer, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, manager, None)
    sink_pv.get_attach_response()
    sink_eet.get_attach_response()

    # Main Loop ---------------------------------------------------------------
    while (enable):
        # Download observed surfaces ------------------------------------------
        sm_manager.get_observed_surfaces()

        # Wait for PV frame ---------------------------------------------------
        sink_pv.acquire()

        # Get PV frame and nearest (in time) EET frame ------------------------
        _, data_pv = sink_pv.get_most_recent_frame()
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        _, data_eet = sink_eet.get_nearest(data_pv.timestamp)
        if ((data_eet is None) or (not hl2ss.is_valid_pose(data_eet.pose))):
            continue

        image = data_pv.payload.image
        eet = hl2ss.unpack_eet(data_eet.payload)

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss.create_pv_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4, dtype=np.float32)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        # Compute world to PV image transformation matrix ---------------------
        world_to_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(pv_extrinsics) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)

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
        cv2.imshow('Video', image)
        cv2.waitKey(1)

    # Stop Spatial Mapping data manager ---------------------------------------
    sm_manager.close()

    # Stop PV and EET streams -------------------------------------------------
    sink_pv.detach()
    sink_eet.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.EXTENDED_EYE_TRACKER)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
