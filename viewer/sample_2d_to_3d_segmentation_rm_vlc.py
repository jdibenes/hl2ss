#------------------------------------------------------------------------------
# 3D segmentation from 2D segmentation using MMDetection. Segmentation is 
# performed on RM VLC video frames. Then, 3D points from RM Depth Long Throw
# are reprojected to the video frames to determine their class.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import cv2
import open3d as o3d
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

# Port
vlc_port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

# Calibration path (must exist but can be empty)
calibration_path = '../calibration/'

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

    # Get RM VLC and RM Depth Long Throw calibration --------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration_vlc = hl2ss_3dcv.get_calibration_rm(calibration_path, host, vlc_port)
    calibration_lt = hl2ss_3dcv.get_calibration_rm(calibration_path, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    
    rotation = hl2ss_3dcv.rm_vlc_get_rotation(vlc_port)
    calibration_vlc.intrinsics, calibration_vlc.extrinsics = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_vlc.intrinsics, calibration_vlc.extrinsics, rotation)

    uv2xy = calibration_lt.uv2xy
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    # Create visualizers ------------------------------------------------------
    cv2.namedWindow('Detections')

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    main_pcd = o3d.geometry.PointCloud()
    first_geometry = True

    # Start RM VLC and RM Depth Long Throw streams ----------------------------
    sink_vlc = hl2ss_mp.stream(hl2ss_lnm.rx_rm_vlc(host, vlc_port))
    sink_depth = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))

    sink_vlc.open()
    sink_depth.open()

    # Init Detector -----------------------------------------------------------
    model = init_detector(config, checkpoint, device=device)
    
    PALETTE = CocoPanopticDataset.PALETTE
    PALETTE.append((0, 0, 0)) # Add color for unrecognized objects

    # Main loop ---------------------------------------------------------------
    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        # Get RM Depth Long Throw frame and nearest (in time) RM VLC frame ----
        _, data_depth = sink_depth.get_most_recent_frame()
        if ((data_depth is None) or (not hl2ss.is_valid_pose(data_depth.pose))):
            continue

        _, data_vlc = sink_vlc.get_nearest(data_depth.timestamp)
        if ((data_vlc is None) or (not hl2ss.is_valid_pose(data_vlc.pose))):
            continue

        # Preprocess frames ---------------------------------------------------
        frame = hl2ss_3dcv.rm_vlc_undistort(data_vlc.payload.image, calibration_vlc.undistort_map)
        frame = hl2ss_3dcv.rm_vlc_rotate_image(frame, rotation)
        frame = hl2ss_3dcv.rm_vlc_to_rgb(frame)

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
        world_to_image = hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_vlc.extrinsics) @ hl2ss_3dcv.camera_to_image(calibration_vlc.intrinsics)
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

    # Stop RM VLC and RM Depth Long Throw streams -----------------------------
    sink_vlc.close()
    sink_depth.close()

    # Stop keyboard events ----------------------------------------------------
    listener.close()
