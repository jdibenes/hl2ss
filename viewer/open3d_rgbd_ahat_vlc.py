#------------------------------------------------------------------------------
# This script demonstrates how to create aligned "rgbd" images, which can be
# used with Open3D, from the depth and grayscale cameras of the HoloLens.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_mp
import hl2ss_3dcv

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration folder
calibration_path = '../calibration'

#------------------------------------------------------------------------------

if __name__ == '__main__':
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    calibration_lf = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, calibration_path)
    calibration_ht = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_AHAT, calibration_path)

    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_ht.intrinsics, hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT)
    xy1 = hl2ss_3dcv.to_homogeneous(uv2xy)
    scale = np.linalg.norm(xy1, axis=2) * (calibration_ht.scale / hl2ss.Parameters_RM_DEPTH_AHAT.FACTOR)

    producer = hl2ss_mp.producer()
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_1, hl2ss.VideoProfile.H264_MAIN, 1*1024*1024)
    producer.configure_rm_depth_ahat(True, host, hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss.ChunkSize.RM_DEPTH_AHAT, hl2ss.StreamMode.MODE_1, hl2ss.VideoProfile.H264_MAIN, 8*1024*1024)
    producer.initialize(hl2ss.StreamPort.RM_VLC_LEFTFRONT, 2*30)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_AHAT, 2*45)
    producer.start(hl2ss.StreamPort.RM_VLC_LEFTFRONT)
    producer.start(hl2ss.StreamPort.RM_DEPTH_AHAT)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()

    sink_lf = consumer.create_sink(producer, hl2ss.StreamPort.RM_VLC_LEFTFRONT, manager, None)
    sink_ht = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_AHAT, manager, ...)

    sink_lf.get_attach_response()
    sink_ht.get_attach_response()

    while (enable):
        sink_ht.acquire()

        data_ht = sink_ht.get_most_recent_frame()
        if ((data_ht is None) or (not hl2ss.is_valid_pose(data_ht.pose))):
            continue

        _, data_lf = sink_lf.get_nearest(data_ht.timestamp)
        if ((data_lf is None) or (not hl2ss.is_valid_pose(data_lf.pose))):
            continue

        depth = hl2ss_3dcv.rm_depth_normalize(data_ht.payload.depth, calibration_ht.undistort_map, scale)
        rgb = cv2.remap(data_lf.payload, calibration_lf.undistort_map[:, :, 0], calibration_lf.undistort_map[:, :, 1], cv2.INTER_LINEAR)
        ht_to_lf_image = hl2ss_3dcv.camera_to_rignode(calibration_ht.extrinsics) @ hl2ss_3dcv.reference_to_world(data_ht.pose) @ hl2ss_3dcv.world_to_reference(data_lf.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lf.extrinsics) @ calibration_lf.intrinsics
        rgb, depth = hl2ss_3dcv.rm_depth_rgbd_registered(depth, rgb, xy1, ht_to_lf_image, cv2.INTER_LINEAR)

        image = np.hstack((depth / np.max(depth), rgb / 255)) # Depth scaled for visibility
        cv2.imshow('RGBD', image)
        cv2.waitKey(1)

    sink_lf.detach()
    sink_ht.detach()
    producer.stop(hl2ss.StreamPort.RM_VLC_LEFTFRONT)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_AHAT)
    listener.join()
