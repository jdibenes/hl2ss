#------------------------------------------------------------------------------
# This script demonstrates how to align RGBD and Scene Understanding data.
# RGBD integration is dynamic.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import multiprocessing as mp
import open3d as o3d
import cv2
import hl2ss
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa
import configparser

# Settings --------------------------------------------------------------------

# HoloLens address
host = config['DEFAULT']['ip']

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# Camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 30
pv_profile = hl2ss.VideoProfile.H265_MAIN
pv_bitrate = hl2ss.get_video_codec_bitrate(pv_width, pv_height, pv_framerate, hl2ss.get_video_codec_default_factor(pv_profile))
lt_png = hl2ss.PngFilterMode.Paeth

# Buffer length in seconds
buffer_size = 10

# Integrator parameters
max_depth = 2.0
voxel_size = 16.0/512.0
block_resolution = 8
block_count = 100000
device = 'cpu:0'
weight_threshold = 0.5

# Scene Understanding manager parameters
world_mesh = True
mesh_lod =  hl2ss.SU_MeshLOD.Fine
query_radius = 0.5
kind_flags = hl2ss.SU_KindFlag.Background | hl2ss.SU_KindFlag.Wall | hl2ss.SU_KindFlag.Floor | hl2ss.SU_KindFlag.Ceiling | hl2ss.SU_KindFlag.Platform | hl2ss.SU_KindFlag.Unknown | hl2ss.SU_KindFlag.World | hl2ss.SU_KindFlag.CompletelyInferred

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

    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().mesh_show_back_face = True

    first_pcd = True

    # Start PV subsystem ------------------------------------------------------
    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Get calibration ---------------------------------------------------------
    calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
    
    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    # Get SU data -------------------------------------------------------------
    su_manager = hl2ss_sa.su_manager(host)
    su_manager.open()
    su_manager.configure(world_mesh, mesh_lod, query_radius, kind_flags)
    su_manager.update()
    su_manager.close()
    items = su_manager.get_items()

    meshes = []
    for item in items.values():
        meshes.extend(item.meshes)

    meshes = [hl2ss_sa.su_mesh_to_open3d_triangle_mesh(mesh) for mesh in meshes]
    for mesh in meshes:
        mesh.compute_vertex_normals()
        vis.add_geometry(mesh)

    # Start streams -----------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate, 'rgb24')
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1, lt_png)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, buffer_size * pv_framerate)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, buffer_size * hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)    
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_lt = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)
    sink_pv.get_attach_response()
    sink_lt.get_attach_response()

    # Create integrator -------------------------------------------------------
    integrator = hl2ss_sa.integrator(voxel_size, block_resolution, block_count, device)
    integrator.set_intrinsics(calibration_lt.intrinsics[:3, :3])
    integrator.set_depth_parameters(1.0, max_depth)

    pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)

    # Main loop ---------------------------------------------------------------
    while (enable):
        # Get frames ----------------------------------------------------------
        sink_lt.acquire()

        _, data_lt = sink_lt.get_most_recent_frame()
        if ((data_lt is None) or (not hl2ss.is_valid_pose(data_lt.pose))):
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        # Integrate -----------------------------------------------------------
        depth = hl2ss_3dcv.rm_depth_undistort(data_lt.payload.depth, calibration_lt.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = data_pv.payload.image

        pv_intrinsics = hl2ss.update_pv_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
        color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)
        
        lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose)
        world_to_lt       = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
        world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics) @ hl2ss_3dcv.camera_to_image(color_intrinsics)
        world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
        pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
        color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR).astype(np.float32) #/ hl2ss._RANGEOF.U8_MAX

        mask_uv = hl2ss_3dcv.slice_to_block((pv_uv[:, :, 0] < 0) | (pv_uv[:, :, 0] >= pv_width) | (pv_uv[:, :, 1] < 0) | (pv_uv[:, :, 1] >= pv_height))
        depth[mask_uv] = 0

        integrator.set_extrinsics(world_to_lt)
        integrator.set_projection(world_to_lt @ calibration_lt.intrinsics)
        integrator.set_depth(depth)
        integrator.set_color(color)

        try:
            integrator.update()
        except:
            pass

        pcd_tmp = integrator.extract_point_cloud(weight_threshold).to_legacy()

        # Update visualization ------------------------------------------------
        if (first_pcd):
            first_pcd = False
            pcd = pcd_tmp
            vis.add_geometry(pcd)
        else:
            pcd.points = pcd_tmp.points
            pcd.colors = pcd_tmp.colors
            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()

    # Stop streams ------------------------------------------------------------
    sink_pv.detach()
    sink_lt.detach()    
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

    # Show final point cloud --------------------------------------------------
    vis.run()
