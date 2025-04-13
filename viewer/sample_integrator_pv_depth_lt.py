#------------------------------------------------------------------------------
# RGBD integration using Open3D. Color information comes from the front RGB
# camera. RGBD images are in PV space and PV resolution.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import open3d as o3d
import time
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
pv_exposure_mode = hl2ss.PV_ExposureMode.Manual
pv_exposure = 5990

# Integration parameters
voxel_length = 1/100
sdf_trunc = 0.04
max_depth = 3.0

# Spatial Mapping manager parameters
show_sm_mesh = False
tpcm = 1000
origin = [0, 0, 0]
radius = 5.0

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

    #uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    uv2xy = calibration_lt.uv2xy
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    xy1_o = xy1[:-1, :-1, :]
    xy1_d = xy1[1:, 1:, :]

    # Create Open3D integrator and visualizer ---------------------------------
    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    first_pcd = True

    # Get SM data -------------------------------------------------------------
    if show_sm_mesh:
        sm_volume = hl2ss.sm_bounding_volume()
        sm_volume.add_sphere(origin, radius)

        sm_manager = hl2ss_sa.sm_manager(host, hl2ss.IPCPort.SPATIAL_MAPPING, triangles_per_cubic_meter=tpcm)
        sm_manager.open()
        sm_manager.set_volumes(sm_volume)
        sm_manager.get_observed_surfaces()
        sm_manager.close()

        meshes = [hl2ss_sa.open3d_triangle_mesh_swap_winding(hl2ss_sa.sm_mesh_to_open3d_triangle_mesh(mesh)) for mesh in sm_manager.get_meshes()]
        for mesh in meshes:
            mesh.compute_vertex_normals()
            vis.add_geometry(mesh)

    # Start PV and RM Depth Long Throw streams --------------------------------
    sink_pv = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, decoded_format='rgb24'))
    sink_depth = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))

    sink_pv.open()
    sink_depth.open()

    last_fs = -1

    # Initialize PV intrinsics and extrinsics ---------------------------------
    pv_intrinsics = hl2ss_3dcv.pv_create_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)
 
    # Main Loop ---------------------------------------------------------------
    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        # Get RM Depth Long Throw frame and nearest (in time) PV frame --------
        fs_lt, data_lt = sink_depth.get_most_recent_frame()
        if ((data_lt is None) or (not hl2ss.is_valid_pose(data_lt.pose))):
            continue

        if (fs_lt <= last_fs):
            time.sleep(1 / hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        last_fs = fs_lt

        # Preprocess frames ---------------------------------------------------
        depth = data_lt.payload.depth
        z     = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = data_pv.payload.image

        # Update PV intrinsics ------------------------------------------------
        # PV intrinsics may change between frames due to autofocus
        pv_intrinsics = hl2ss_3dcv.pv_update_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
        color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)
        
        # Generate aligned RGBD image -----------------------------------------
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

        # Convert to Open3D RGBD image ----------------------------------------
        color_image = o3d.geometry.Image(color)
        depth_image = o3d.geometry.Image(pv_z)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
        intrinsics_pv = o3d.camera.PinholeCameraIntrinsic(pv_width, pv_height, color_intrinsics[0, 0], color_intrinsics[1, 1], color_intrinsics[2, 0], color_intrinsics[2, 1])

        # Integrate RGBD and display point cloud ------------------------------
        volume.integrate(rgbd, intrinsics_pv, world_to_pv.transpose())
        pcd_tmp = volume.extract_point_cloud()

        if (first_pcd):
            first_pcd = False
            pcd = pcd_tmp
            vis.add_geometry(pcd)
        else:
            pcd.points = pcd_tmp.points
            pcd.colors = pcd_tmp.colors
            vis.update_geometry(pcd)

    # Stop PV and RM Depth Long Throw streams ---------------------------------
    sink_pv.close()
    sink_depth.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.close()

    # Show final point cloud --------------------------------------------------
    vis.run()
