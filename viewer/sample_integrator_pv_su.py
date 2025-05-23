#------------------------------------------------------------------------------
# This script demonstrates how to align RGBD and Scene Understanding data.
# RGBD integration is dynamic.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import open3d as o3d
import time
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# Camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 30

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
query_radius = 1.0
kind_flags = hl2ss.SU_KindFlag.Wall | hl2ss.SU_KindFlag.Floor | hl2ss.SU_KindFlag.Ceiling | hl2ss.SU_KindFlag.Platform | hl2ss.SU_KindFlag.World

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()

    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    first_pcd = True

    # Start PV subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Get calibration ---------------------------------------------------------
    calibration_lt = hl2ss_3dcv.get_calibration_rm(calibration_path, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    
    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    # Get SU data -------------------------------------------------------------
    task = hl2ss.su_task(False, True, True, world_mesh, mesh_lod, query_radius, hl2ss.SU_Create.New, kind_flags, False, False, True, False, True, False, [])

    su_manager = hl2ss_sa.su_manager(host, hl2ss.IPCPort.SCENE_UNDERSTANDING)
    su_manager.open()
    su_manager.update(task)
    su_manager.close()
    items = su_manager.get_items()

    meshes = []
    for item in items.values():
        meshes.extend(item.meshes)

    meshes = [hl2ss_sa.open3d_triangle_mesh_swap_winding(hl2ss_sa.su_mesh_to_open3d_triangle_mesh(mesh)) for mesh in meshes]
    for mesh in meshes:
        mesh.compute_vertex_normals()
        vis.add_geometry(mesh)

    # Start streams -----------------------------------------------------------
    sink_pv = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, decoded_format='rgb24'))
    sink_lt = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))

    sink_pv.open()
    sink_lt.open()

    last_fs = -1

    # Create integrator -------------------------------------------------------
    integrator = hl2ss_sa.integrator(voxel_size, block_resolution, block_count, device)
    integrator.set_intrinsics(calibration_lt.intrinsics[:3, :3])
    integrator.set_depth_parameters(1.0, max_depth)

    pv_intrinsics = hl2ss_3dcv.pv_create_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)

    # Main loop ---------------------------------------------------------------
    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        # Get frames ----------------------------------------------------------
        fs_lt, data_lt = sink_lt.get_most_recent_frame()
        if ((data_lt is None) or (not hl2ss.is_valid_pose(data_lt.pose))):
            continue

        if (fs_lt <= last_fs):
            time.sleep(1 / hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        last_fs = fs_lt

        # Integrate -----------------------------------------------------------
        depth = hl2ss_3dcv.rm_depth_undistort(data_lt.payload.depth, calibration_lt.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = data_pv.payload.image

        pv_intrinsics = hl2ss_3dcv.pv_update_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
        color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)
        
        lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose)
        world_to_lt       = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
        world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics) @ hl2ss_3dcv.camera_to_image(color_intrinsics)
        world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
        pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
        color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR).astype(np.float32)

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

    # Stop streams ------------------------------------------------------------
    sink_pv.close()
    sink_lt.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.close()

    # Show final point cloud --------------------------------------------------
    vis.run()
