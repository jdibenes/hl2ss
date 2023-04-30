
from pynput import keyboard

import multiprocessing as mp
import numpy as np
import cv2
import hl2ss
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa
import hl2ss_utilities

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

# EET parameters
eet_fps = 30

# Marker properties
radius = 5
combined_color = (255, 0, 255)
left_color = (0, 0, 255)
right_color = (255, 0, 0)
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

    sm_manager = hl2ss_sa.sm_manager(host, triangles_per_cubic_meter, mesh_threads)
    sm_manager.open()
    sm_manager.set_volumes(volumes)
    sm_manager.get_observed_surfaces()

    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    producer = hl2ss_mp.producer()
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate, 'bgr24')
    producer.configure_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss.ChunkSize.EXTENDED_EYE_TRACKER, eet_fps)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, framerate * buffer_length)
    producer.initialize(hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.EXTENDED_EYE_TRACKER)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, ...)
    sink_eet = consumer.create_sink(producer, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, manager, None)
    sink_pv.get_attach_response()
    sink_eet.get_attach_response()

    while (enable):
        sm_manager.get_observed_surfaces()

        sink_pv.acquire()

        _, data_pv = sink_pv.get_most_recent_frame()
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        _, data_eet = sink_eet.get_nearest(data_pv.timestamp)
        if ((data_eet is None) or (not hl2ss.is_valid_pose(data_eet.pose))):
            continue

        image = data_pv.payload.image
        eet = hl2ss.unpack_eet(data_eet.payload)

        pv_intrinsics = hl2ss.create_pv_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4, dtype=np.float32)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        world_to_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(pv_extrinsics) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)

        if (eet.left_ray_valid):
            left_ray = eet.left_ray

            left_ray.origin = hl2ss_3dcv.transform(left_ray.origin.reshape((1, 3)), data_eet.pose)
            left_ray.direction = hl2ss_3dcv.orient(left_ray.direction.reshape((1, 3)), data_eet.pose)

            d = sm_manager.cast_rays(hl2ss_utilities.si_ray_to_vector(left_ray.origin, left_ray.direction))
            if (np.isfinite(d)):
                left_point = hl2ss_utilities.si_ray_to_point(left_ray.origin, left_ray.direction, d)
                project_points(image, left_point, world_to_image, radius, left_color, thickness)

        if (eet.right_ray_valid):
            right_ray = eet.right_ray

            right_ray.origin = hl2ss_3dcv.transform(right_ray.origin.reshape((1, 3)), data_eet.pose)
            right_ray.direction = hl2ss_3dcv.orient(right_ray.direction.reshape((1, 3)), data_eet.pose)

            d = sm_manager.cast_rays(hl2ss_utilities.si_ray_to_vector(right_ray.origin, right_ray.direction))
            if (np.isfinite(d)):
                right_point = hl2ss_utilities.si_ray_to_point(right_ray.origin, right_ray.direction, d)
                project_points(image, right_point, world_to_image, radius, right_color, thickness)

        if (eet.combined_ray_valid):
            combined_ray = eet.combined_ray

            combined_ray.origin = hl2ss_3dcv.transform(combined_ray.origin.reshape((1, 3)), data_eet.pose)
            combined_ray.direction = hl2ss_3dcv.orient(combined_ray.direction.reshape((1, 3)), data_eet.pose)

            d = sm_manager.cast_rays(hl2ss_utilities.si_ray_to_vector(combined_ray.origin, combined_ray.direction))
            if (np.isfinite(d)):
                combined_point = hl2ss_utilities.si_ray_to_point(combined_ray.origin, combined_ray.direction, d)
                project_points(image, combined_point, world_to_image, radius, combined_color, thickness)

        cv2.imshow('Video', image)
        cv2.waitKey(1)

    sm_manager.close()

    sink_pv.detach()
    sink_eet.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.EXTENDED_EYE_TRACKER)
    listener.join()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
