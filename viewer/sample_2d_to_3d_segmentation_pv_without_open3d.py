#------------------------------------------------------------------------------
# 3D segmentation from 2D segmentation using MMDetection. Segmentation is 
# performed on PV video frames. Then, 3D points from RM Depth Long Throw are
# reprojected to the video frames to determine their class.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import cv2
import open3d as o3d
import mmcv
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import configparser

from mmdet.apis import inference_detector, init_detector
from mmdet.registry import VISUALIZERS
from mmdet.datasets import CocoPanopticDataset
INSTANCE_OFFSET = 81
# Settings --------------------------------------------------------------------
config = configparser.ConfigParser()
config.read('config.ini')
# HoloLens address
host = config['DEFAULT']['ip']

# Calibration path (must exist but can be empty)
calibration_path = '../calibration/'

# Camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 15

# Buffer length in seconds
buffer_length = 5

# Maximum depth, points beyond are removed
max_depth = 2

# MMDetection parameters
config = './mmdetection/configs/rtmdet/rtmdet_tiny_8xb32-300e_coco.py'
#'./mmdetection/configs/mask2former/mask2former_swin-t-p4-w7-224_8xb2-lsj-50e_coco-panoptic.py'
checkpoint = './rtmdet_tiny_8xb32-300e_coco_20220902_112414-78e30dcc.pth'
#'./mask2former_swin-t-p4-w7-224_8xb2-lsj-50e_coco-panoptic_20220326_224553-3ec9e0ae.pth'
device = 'cuda'
score_thr = 0.3
wait_time = 1

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Start PV Subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
    cv2.namedWindow('Detections')
    # Start PV and RM Depth Long Throw streams --------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, buffer_length * pv_framerate)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, buffer_length * hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_depth = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, None)
    sink_pv.get_attach_response()
    sink_depth.get_attach_response()
    # Init Detector -----------------------------------------------------------
    model = init_detector(config, checkpoint, device=device)
    visualizer = VISUALIZERS.build(model.cfg.visualizer)
    # The dataset_meta is loaded from the checkpoint and
    # then pass to the model in init_detector
    visualizer.dataset_meta = model.dataset_meta
     
    # Main loop ---------------------------------------------------------------
    while (enable):
        # Get RM Depth Long Throw frame and nearest (in time) PV frame --------
        _, data_depth = sink_depth.get_most_recent_frame()
        if ((data_depth is None) or (not hl2ss.is_valid_pose(data_depth.pose))):
            continue

        _, data_pv = sink_pv.get_nearest(data_depth.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss.create_pv_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4, dtype=np.float32)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        # Preprocess frames ---------------------------------------------------
        frame = data_pv.payload.image
        # Inference -----------------------------------------------------------
        result = inference_detector(model, frame)
        print(result)
        #mask = result.pred_panoptic_seg.sem_seg.detach().cpu().numpy()
        #mask = result.pred_instances.priors.detach().cpu().numpy()
        # Build pointcloud ----------------------------------------------------
        #points = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
    
        try:
            visualizer.add_datasample(
                name='Detections',
                image=frame,
                data_sample=result,
                draw_gt=False,
                show=False)
            frame = visualizer.get_image()    
            mmcv.imshow(frame, 'Detections', wait_time)
        except:
            print("fail")
        # main_pcd.points = o3d.utility.Vector3dVector(points)
        # main_pcd.colors = o3d.utility.Vector3dVector(colors)

        # if (first_geometry):
        #    vis.add_geometry(main_pcd)
        #    first_geometry = False
        # else:
        #    vis.update_geometry(main_pcd)

        # vis.poll_events()
        # vis.update_renderer()

    # Stop PV and RM Depth Long Throw streams ---------------------------------
    sink_pv.detach()
    sink_depth.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
