
# 3D segmentation from 2d segmentation using mmdetection

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import hl2ss
import hl2ss_utilities
import hl2ss_mp
import hl2ss_3dcv
import cv2
import mmcv
import os
import open3d as o3d

from mmdet.apis import inference_detector, init_detector
from mmdet.core import INSTANCE_OFFSET

host = '192.168.1.7'
calibration_path = '../calibration/'
config = 'C:/Users/jcds/Documents/GitHub/mmdetection/configs/mask2former/mask2former_swin-t-p4-w7-224_lsj_8x2_50e_coco-panoptic.py'
checkpoint = 'C:/Users/jcds/Documents/GitHub/mmdetection/demo/mask2former_swin-t-p4-w7-224_lsj_8x2_50e_coco-panoptic_20220326_224553-fc567107.pth'
device = 'cuda:0'
score_thr = 0.3
wait_time = 1
buffer_length = 5
max_depth = 1.5

CLASSES = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
        ' truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
        'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
        'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
        'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
        'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
        'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
        'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
        'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
        'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
        'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
        'scissors', 'teddy bear', 'hair drier', 'toothbrush', 'banner',
        'blanket', 'bridge', 'cardboard', 'counter', 'curtain', 'door-stuff',
        'floor-wood', 'flower', 'fruit', 'gravel', 'house', 'light',
        'mirror-stuff', 'net', 'pillow', 'platform', 'playingfield',
        'railroad', 'river', 'road', 'roof', 'sand', 'sea', 'shelf', 'snow',
        'stairs', 'tent', 'towel', 'wall-brick', 'wall-stone', 'wall-tile',
        'wall-wood', 'water-other', 'window-blind', 'window-other',
        'tree-merged', 'fence-merged', 'ceiling-merged', 'sky-other-merged',
        'cabinet-merged', 'table-merged', 'floor-other-merged',
        'pavement-merged', 'mountain-merged', 'grass-merged', 'dirt-merged',
        'paper-merged', 'food-other-merged', 'building-other-merged',
        'rock-merged', 'wall-other-merged', 'rug-merged'
    ]

PALETTE = [(220, 20, 60), (119, 11, 32), (0, 0, 142), (0, 0, 230),
               (106, 0, 228), (0, 60, 100), (0, 80, 100), (0, 0, 70),
               (0, 0, 192), (250, 170, 30), (100, 170, 30), (220, 220, 0),
               (175, 116, 175), (250, 0, 30), (165, 42, 42), (255, 77, 255),
               (0, 226, 252), (182, 182, 255), (0, 82, 0), (120, 166, 157),
               (110, 76, 0), (174, 57, 255), (199, 100, 0), (72, 0, 118),
               (255, 179, 240), (0, 125, 92), (209, 0, 151), (188, 208, 182),
               (0, 220, 176), (255, 99, 164), (92, 0, 73), (133, 129, 255),
               (78, 180, 255), (0, 228, 0), (174, 255, 243), (45, 89, 255),
               (134, 134, 103), (145, 148, 174), (255, 208, 186),
               (197, 226, 255), (171, 134, 1), (109, 63, 54), (207, 138, 255),
               (151, 0, 95), (9, 80, 61), (84, 105, 51), (74, 65, 105),
               (166, 196, 102), (208, 195, 210), (255, 109, 65), (0, 143, 149),
               (179, 0, 194), (209, 99, 106), (5, 121, 0), (227, 255, 205),
               (147, 186, 208), (153, 69, 1), (3, 95, 161), (163, 255, 0),
               (119, 0, 170), (0, 182, 199), (0, 165, 120), (183, 130, 88),
               (95, 32, 0), (130, 114, 135), (110, 129, 133), (166, 74, 118),
               (219, 142, 185), (79, 210, 114), (178, 90, 62), (65, 70, 15),
               (127, 167, 115), (59, 105, 106), (142, 108, 45), (196, 172, 0),
               (95, 54, 80), (128, 76, 255), (201, 57, 1), (246, 0, 122),
               (191, 162, 208), (255, 255, 128), (147, 211, 203),
               (150, 100, 100), (168, 171, 172), (146, 112, 198),
               (210, 170, 100), (92, 136, 89), (218, 88, 184), (241, 129, 0),
               (217, 17, 255), (124, 74, 181), (70, 70, 70), (255, 228, 255),
               (154, 208, 0), (193, 0, 92), (76, 91, 113), (255, 180, 195),
               (106, 154, 176),
               (230, 150, 140), (60, 143, 255), (128, 64, 128), (92, 82, 55),
               (254, 212, 124), (73, 77, 174), (255, 160, 98), (255, 255, 255), #
               (104, 84, 109), (169, 164, 131), (225, 199, 255), (137, 54, 74),
               (135, 158, 223), (7, 246, 231), (107, 255, 200), (58, 41, 149),
               (183, 121, 142), (255, 73, 97), (107, 142, 35), (190, 153, 153),
               (146, 139, 141),
               (70, 130, 180), (134, 199, 156), (209, 226, 140), (96, 36, 108),
               (96, 96, 96), (64, 170, 64), (152, 251, 152), (208, 229, 228),
               (206, 186, 171), (152, 161, 64), (116, 112, 0), (0, 114, 143),
               (102, 102, 156), (250, 141, 255),
               (0,0,0)]


def main():
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    classes_ignore = ['dining table', 'table-merged', 'wall-brick', 'wall-stone', 'wall-tile', 'wall-wood', 'wall-other-merged', 'ceiling-merged', 'floor-wood', 'floor-other-merged']

    label_ignore = [i for i in range(0, len(CLASSES)) if CLASSES[i] in classes_ignore]

    calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
    calibration_vlc = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, calibration_path)

    rotation = hl2ss_3dcv.rm_vlc_get_rotation(hl2ss.StreamPort.RM_VLC_LEFTFRONT)
    calibration_vlc.intrinsics, calibration_vlc.extrinsics = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_vlc.intrinsics, calibration_vlc.extrinsics, rotation)

    uv2xy = calibration_lt.uv2xy
    xy1, scale, depth_to_vlc_image = hl2ss_3dcv.rm_depth_registration(uv2xy, calibration_lt.scale, calibration_lt.extrinsics, calibration_vlc.intrinsics, calibration_vlc.extrinsics)

    producer = hl2ss_mp.producer()
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, hl2ss.VideoProfile.H264_BASE, 1024*1024)
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1, hl2ss.PngFilterMode.Paeth)
    producer.initialize(hl2ss.StreamPort.RM_VLC_LEFTFRONT, buffer_length*hl2ss.Parameters_RM_VLC.FPS)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, buffer_length*hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)
    producer.start(hl2ss.StreamPort.RM_VLC_LEFTFRONT)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sink_vlc = consumer.create_sink(producer, hl2ss.StreamPort.RM_VLC_LEFTFRONT, manager, None)
    sink_depth = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)

    sinks = [sink_vlc, sink_depth]
    
    [sink.get_attach_response() for sink in sinks]

    model = init_detector(config, checkpoint, device=device)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    first_geometry = True

    while (enable):
        sink_depth.acquire()

        vis.poll_events()
        vis.update_renderer()

        data_depth = sink_depth.get_most_recent_frame()
        if (not hl2ss.is_valid_pose(data_depth.pose)):
            continue

        _, data_vlc = sink_vlc.get_nearest(data_depth.timestamp)
        if (data_vlc is None):
            continue

        frame = cv2.remap(data_vlc.payload, calibration_vlc.undistort_map[:, :, 0], calibration_vlc.undistort_map[:, :, 1], cv2.INTER_LINEAR)
        frame = hl2ss_3dcv.rm_vlc_rotate_image(frame, rotation)
        frame = hl2ss_3dcv.rm_vlc_to_rgb(frame)

        result = inference_detector(model, frame)
        mask = result['pan_results']

        depth = hl2ss_3dcv.rm_depth_scale(data_depth.payload.depth, scale)
        depth[depth > max_depth] = 0

        points = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
        hpoints = hl2ss_3dcv.to_homogeneous(points)

        pixels, _, = hl2ss_3dcv.project_to_image(hpoints, depth_to_vlc_image)
        map_u = pixels[:, 0].reshape(depth.shape)
        map_v = pixels[:, 1].reshape(depth.shape)

        depth_to_world = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_depth.pose)
        hwpoints = hpoints @ depth_to_world
        points, _ = hl2ss_3dcv.to_inhomogeneous(hwpoints)

        labels = cv2.remap(mask, map_u, map_v, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=model.num_classes).reshape((-1, 1))
        rgb = cv2.remap(frame, map_u, map_v, cv2.INTER_NEAREST)

        kinds = labels % INSTANCE_OFFSET
        instances = labels // INSTANCE_OFFSET

        colors = np.array([list(PALETTE[kinds[i, 0]]) for i in range(0, labels.size)], dtype=np.uint8)
        colors = np.hstack((colors[:, 0].reshape((-1, 1)), colors[:, 1].reshape((-1, 1)), colors[:, 2].reshape((-1, 1))))

        rgb = np.hstack((rgb[:, :, 0].reshape((-1, 1)), rgb[:, :, 1].reshape((-1, 1)), rgb[:, :, 2].reshape((-1, 1))))

        colors = 0.5 * colors + 0.5 * rgb

        select = (depth.reshape((-1, 1)) > 0)
        select = select.reshape((-1,))

        points = points[select, :]
        colors = colors[select, :]

        main_pcd = o3d.geometry.PointCloud()
        main_pcd.points = o3d.utility.Vector3dVector(points)
        main_pcd.colors = o3d.utility.Vector3dVector(colors / 255)
        
        geometries = [main_pcd]

        if (not first_geometry):
            for geometry in prev_geometries:
                vis.remove_geometry(geometry, False)

        for geometry in geometries:
            vis.add_geometry(geometry, first_geometry)

        first_geometry = False
        prev_geometries = geometries

        vis.poll_events()
        vis.update_renderer()

        detections = model.show_result(frame, result, score_thr=score_thr, mask_color=PALETTE)
        cv2.namedWindow('Detections', 0)
        mmcv.imshow(detections, 'Detections', wait_time)

    cv2.destroyAllWindows()

    [sink.detach() for sink in sinks]
    producer.stop()
    listener.join()

if __name__ == '__main__':
    main()
