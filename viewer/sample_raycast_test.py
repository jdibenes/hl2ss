
import multiprocessing as mp
import numpy as np
import cv2
import hl2ss
import hl2ss_mp
import hl2ss_3dcv

host = '192.168.1.7'
width = 760
height = 428
framerate = 30
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = hl2ss.get_video_codec_bitrate(width, height, framerate, hl2ss.get_video_codec_default_factor(profile))

if (__name__ == '__main__'):
    sm_manager = hl2ss_3dcv.sm_manager(host, 1000, 2)
    sm_manager.open()
    volumes = hl2ss.sm_bounding_volume()
    volumes.add_sphere([0,0,0], 5)
    sm_manager.create_observer()
    sm_manager.set_volumes(volumes)
    sm_manager.update()
    sm_manager.close()

    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    producer = hl2ss_mp.producer()
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate, 'bgr24')
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, 300)
    producer.initialize(hl2ss.StreamPort.SPATIAL_INPUT, 300)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.SPATIAL_INPUT)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, manager, None)
    sink_pv.get_attach_response()
    sink_si.get_attach_response()

    frame = np.zeros((height, width, 3), dtype=np.uint8)

    while (True):
        cv2.imshow('Video', frame)

        key = cv2.waitKey(1) & 0xFF
        if (key == 27):
            break

        _, data_pv = sink_pv.get_most_recent_frame()
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        _, data_si = sink_si.get_nearest(data_pv.timestamp)
        if (data_si is None):
            continue

        frame = data_pv.payload.image

        pv_intrinsics = hl2ss.create_pv_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(pv_extrinsics) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)

        si = hl2ss.unpack_si(data_si.payload)

        if (not si.is_valid_eye_ray()):
            continue

        ray = si.get_eye_ray()
        ray = np.hstack((ray.origin.reshape((1, -1)), ray.direction.reshape((1, -1))))

        d = sm_manager.cast_rays(ray)
        if (np.isinf(d)):
            continue

        point = ray[0, 0:3] + d*ray[0, 3:6]
        image_point = hl2ss_3dcv.project(point, world_to_pv_image)
        x = int(image_point[0])
        y = int(image_point[1])
        if (x < 0 or y < 0 or x >= width or y >= height):
            continue

        frame = cv2.circle(frame, (x, y), 5, (255, 0, 255), -1)

    sink_pv.detach()
    sink_si.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.SPATIAL_INPUT)

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
