#------------------------------------------------------------------------------
# 3D segmentation from 2D segmentation using MMDetection. Segmentation is 
# performed on PV video frames. Then, 3D points from RM Depth Long Throw are
# reprojected to the video frames to determine their class.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import open3d as o3d
import cv2
import mmcv
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_utilities

from mmdet.apis import inference_detector, init_detector
from mmdet.core import INSTANCE_OFFSET
from mmdet.datasets import CocoPanopticDataset

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration path (must exist but can be empty)
calibration_path = '../calibration/'

# Camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 15

# Maximum depth, points beyond are removed
max_depth = 2

# MMDetection parameters
config = 'C:/Users/jcds/Documents/GitHub/mmdetection/configs/mask2former/mask2former_swin-t-p4-w7-224_lsj_8x2_50e_coco-panoptic.py'
checkpoint = 'C:/Users/jcds/Documents/GitHub/mmdetection/demo/mask2former_swin-t-p4-w7-224_lsj_8x2_50e_coco-panoptic_20220326_224553-fc567107.pth'
device = 'cuda:0'
score_thr = 0.3
wait_time = 1

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()

    # Start PV Subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Get RM Depth Long Throw calibration -------------------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration_lt = hl2ss_3dcv.get_calibration_rm(calibration_path, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    
    uv2xy = calibration_lt.uv2xy
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    # Create visualizers ------------------------------------------------------
    cv2.namedWindow('Detections')

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    main_pcd = o3d.geometry.PointCloud()
    first_geometry = True

    # Start PV and RM Depth Long Throw streams --------------------------------

    sink_pv = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate))
    sink_lt = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))

    sink_pv.open()
    sink_lt.open()

    # Init Detector -----------------------------------------------------------
    model = init_detector(config, checkpoint, device=device)
    
    PALETTE = CocoPanopticDataset.PALETTE
    PALETTE.append((0, 0, 0)) # Add color for unrecognized objects

    # Main loop ---------------------------------------------------------------
    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        # Get RM Depth Long Throw frame and nearest (in time) PV frame --------
        _, data_depth = sink_lt.get_most_recent_frame()
        if ((data_depth is None) or (not hl2ss.is_valid_pose(data_depth.pose))):
            continue

        _, data_pv = sink_pv.get_nearest(data_depth.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss_3dcv.pv_create_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
        pv_extrinsics = np.eye(4, 4, dtype=np.float32)
        pv_intrinsics, pv_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        # Preprocess frames ---------------------------------------------------
        frame = data_pv.payload.image
        depth = hl2ss_3dcv.rm_depth_normalize(data_depth.payload.depth, scale)
        depth[depth > max_depth] = 0

        # Inference -----------------------------------------------------------
        result = inference_detector(model, frame)
        mask = result['pan_results']

        # Build pointcloud ----------------------------------------------------
        points = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
        depth_to_world = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_depth.pose)
        points = hl2ss_3dcv.transform(points, depth_to_world)

        # Project pointcloud image --------------------------------------------
        world_to_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(pv_extrinsics) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)
        pixels = hl2ss_3dcv.project(points, world_to_image)
        
        map_u = pixels[:, :, 0]
        map_v = pixels[:, :, 1]

        # Get 3D points labels and colors -------------------------------------
        labels = cv2.remap(mask, map_u, map_v, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=model.num_classes)
        rgb = cv2.remap(frame, map_u, map_v, cv2.INTER_NEAREST)

        points = hl2ss_3dcv.block_to_list(points)
        labels = labels.reshape((-1,))
        rgb = hl2ss_3dcv.block_to_list(rgb)
        
        # Get class colors ----------------------------------------------------
        kinds = labels % INSTANCE_OFFSET
        instances = labels // INSTANCE_OFFSET

        class_colors = np.array([list(PALETTE[kind]) for kind in kinds], dtype=np.uint8)

        # Get final color -----------------------------------------------------
        colors = (0.5 * class_colors) + (0.5 * rgb)

        # Remove invalid points -----------------------------------------------
        select = depth.reshape((-1,)) > 0

        points = points[select, :]
        colors = colors[select, :] / 255

        # Visualize results ---------------------------------------------------
        detections = model.show_result(frame, result, score_thr=score_thr, mask_color=PALETTE)
        mmcv.imshow(detections, 'Detections', wait_time)

        main_pcd.points = o3d.utility.Vector3dVector(points)
        main_pcd.colors = o3d.utility.Vector3dVector(colors)

        if (first_geometry):
            vis.add_geometry(main_pcd)
            first_geometry = False
        else:
            vis.update_geometry(main_pcd)

    # Stop PV and RM Depth Long Throw streams ---------------------------------
    sink_pv.close()
    sink_lt.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.close()
