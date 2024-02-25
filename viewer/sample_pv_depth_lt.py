#------------------------------------------------------------------------------
# Experimental depth-to-PV RGBD generation via zero order hold.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import open3d as o3d
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# Front RGB camera parameters
pv_width = 640
pv_height = 360
pv_fps = 30

# Buffer length in seconds
buffer_size = 10

# Maximum depth in meters
max_depth = 3.0

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

    # Get RM Depth Long Throw calibration -------------------------------------
    # Calibration data will be downloaded if it's not in the calibration folder
    calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
    
    uv2xy = calibration_lt.uv2xy #hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    xy1_o = xy1[:-1, :-1, :]
    xy1_d = xy1[1:, 1:, :]

    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    first_pcd = True

    # Start PV and RM Depth Long Throw streams --------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_fps))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, pv_fps * buffer_size)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS * buffer_size)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_lt = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)
    
    sink_pv.get_attach_response()
    sink_lt.get_attach_response()

    # Initialize PV intrinsics and extrinsics ---------------------------------
    pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)

    VI = hl2ss_utilities.framerate_counter()
    VI.reset()

    # Main Loop ---------------------------------------------------------------
    while (enable):
        # Wait for RM Depth Long Throw frame ----------------------------------
        sink_lt.acquire()

        # Get RM Depth Long Throw frame and nearest (in time) PV frame --------
        _, data_lt = sink_lt.get_most_recent_frame()
        if ((data_lt is None) or (not hl2ss.is_valid_pose(data_lt.pose))):
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue
        
        # Preprocess frames ---------------------------------------------------
        depth = data_lt.payload.depth #hl2ss_3dcv.rm_depth_undistort(data_lt.payload.depth, calibration_lt.undistort_map)
        z     = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = data_pv.payload.image

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss.update_pv_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
        color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        # Generate depth map for PV image -------------------------------------
        lt_to_world    = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose)
        world_to_pv    = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics)
        pv_to_pv_image = hl2ss_3dcv.camera_to_image(color_intrinsics)

        lt_points_o    = hl2ss_3dcv.rm_depth_to_points(xy1_o, z[:-1, :-1, :])        
        world_points_o = hl2ss_3dcv.transform(lt_points_o, lt_to_world)
        pv_points_o    = hl2ss_3dcv.transform(world_points_o, world_to_pv)
        pv_depth       = pv_points_o[:, :, 2:]
        pv_uv_o        = hl2ss_3dcv.project(pv_points_o, pv_to_pv_image)

        lt_points_d    = hl2ss_3dcv.rm_depth_to_points(xy1_d, z[:-1, :-1, :])
        world_points_d = hl2ss_3dcv.transform(lt_points_d, lt_to_world)
        pv_uv_d        = hl2ss_3dcv.project(world_points_d, world_to_pv @ pv_to_pv_image)

        pv_list_o     = hl2ss_3dcv.block_to_list(pv_uv_o)
        pv_list_d     = hl2ss_3dcv.block_to_list(pv_uv_d)
        pv_list_depth = hl2ss_3dcv.block_to_list(pv_depth)

        mask = (depth[:-1,:-1].reshape((-1,)) > 0)

        pv_list = np.hstack((np.floor(pv_list_o[mask, :]), np.floor(pv_list_d[mask, :]) + 1, pv_list_depth[mask]))
        pv_z    = np.zeros((pv_height, pv_width), dtype=np.float32)

        for n in range(0, pv_list.shape[0]):
            u0 = int(pv_list[n, 0])
            v0 = int(pv_list[n, 1])
            u1 = int(pv_list[n, 2])
            v1 = int(pv_list[n, 3])

            if ((u0 < 0) or (u0 >= pv_width)):
                continue
            if ((u1 < 0) or (u1 > pv_width)):
                continue
            if ((v0 < 0) or (v0 >= pv_height)):
                continue
            if ((v1 < 0) or (v1 > pv_height)):
                continue

            pv_z[v0:v1, u0:u1] = pv_list[n, 4]

        # FPS -----------------------------------------------------------------
        # ~5 FPS for 1920x1080
        VI.increment()
        if (VI.delta() >= 2):
            print(f'FPS: {VI.get()}')
            reset_VI = True
        else:
            reset_VI = False

        # Display RGBD pair ---------------------------------------------------
        cv2.imshow('RGB', color)
        cv2.imshow('Depth', pv_z / max_depth) # scale for visibility
        cv2.waitKey(1)

        # Convert to Open3D RGBD image and create pointcloud ------------------
        color_image = o3d.geometry.Image(color[:,:,::-1].copy())
        depth_image = o3d.geometry.Image(pv_z)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)

        o3d_pv_intrinsics = o3d.camera.PinholeCameraIntrinsic(pv_width, pv_height, color_intrinsics[0, 0], color_intrinsics[1, 1], color_intrinsics[2, 0], color_intrinsics[2, 1])
        tmp_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_pv_intrinsics)

        # Display pointcloud --------------------------------------------------
        pcd.points = tmp_pcd.points
        pcd.colors = tmp_pcd.colors

        if (first_pcd):
            vis.add_geometry(pcd)
            first_pcd = False
        else:
            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()

        if (reset_VI):
            VI.reset()

    # Stop PV and RM Depth Long Throw streams ---------------------------------
    sink_pv.detach()
    sink_lt.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()
