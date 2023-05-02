#------------------------------------------------------------------------------
# This script receives video frames and spatial input data from the HoloLens.
# The received head pointer, hand joint positions and gaze point are projected
# onto the video frame.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_utilities
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa

# Settings --------------------------------------------------------------------

# HoloLens 2 address
host = "192.168.1.7"

# Camera parameters
# See etc/hl2_capture_formats.txt for a list of supported formats
width     = 760
height    = 428
framerate = 30

# Video Encoding profiles
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = hl2ss.get_video_codec_bitrate(width, height, framerate, hl2ss.get_video_codec_default_factor(profile))

# Marker properties
radius = 5
head_color = (  0,   0, 255)
hand_color = (  0, 255,   0)
gaze_color = (255,   0, 255)
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
    def project_points(image, points, P, radius, color, thickness):
        for x, y in hl2ss_3dcv.project(points, P):
            ix, iy = int(x), int(y)
            if (ix >= 0 and iy >= 0 and ix < image.shape[1] and iy < image.shape[0]):
                cv2.circle(image, (ix, iy), radius, color, thickness)

    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    volumes = hl2ss.sm_bounding_volume()
    volumes.add_sphere(sphere_center, sphere_radius)

    sm_manager = hl2ss_sa.sm_mp_manager(host, triangles_per_cubic_meter, mesh_threads)
    sm_manager.open()
    sm_manager.set_volumes(volumes)
    sm_manager.get_observed_surfaces()
    
    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    producer = hl2ss_mp.producer()
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate, 'bgr24')
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, framerate * buffer_length)
    producer.initialize(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.SPATIAL_INPUT)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, ...)
    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, manager, None)
    sink_pv.get_attach_response()
    sink_si.get_attach_response()

    while (enable):
        sm_manager.get_observed_surfaces()

        sink_pv.acquire()

        _, data_pv = sink_pv.get_most_recent_frame()
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        _, data_si = sink_si.get_nearest(data_pv.timestamp)
        if (data_si is None):
            continue

        image = data_pv.payload.image
        si = hl2ss.unpack_si(data_si.payload)

        pv_intrinsics = hl2ss.create_pv_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4, dtype=np.float32)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        world_to_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(pv_extrinsics) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)

        if (si.is_valid_head_pose()):
            head_pose = si.get_head_pose()
            d = sm_manager.cast_rays(hl2ss_utilities.si_ray_to_vector(head_pose.position, head_pose.forward))
            if (np.isfinite(d)):
                head_point = hl2ss_utilities.si_ray_to_point(head_pose.position, head_pose.forward, d)
                project_points(image, head_point, world_to_image, radius, head_color, thickness)

        if (si.is_valid_hand_left()):
            left_points = hl2ss_utilities.si_unpack_hand(si.get_hand_left()).positions
            project_points(image, left_points, world_to_image, radius, hand_color, thickness)
        
        if (si.is_valid_hand_right()):
            right_points = hl2ss_utilities.si_unpack_hand(si.get_hand_right()).positions
            project_points(image, right_points, world_to_image, radius, hand_color, thickness)

        if (si.is_valid_eye_ray()):
            eye_ray = si.get_eye_ray()
            d = sm_manager.cast_rays(hl2ss_utilities.si_ray_to_vector(eye_ray.origin, eye_ray.direction))
            if (np.isfinite(d)):
                gaze_point = hl2ss_utilities.si_ray_to_point(eye_ray.origin, eye_ray.direction, d)
                project_points(image, gaze_point, world_to_image, radius, gaze_color, thickness)

        cv2.imshow('Video', image)
        cv2.waitKey(1)
        
    sm_manager.close()

    sink_pv.detach()
    sink_si.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.SPATIAL_INPUT)
    listener.join()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

