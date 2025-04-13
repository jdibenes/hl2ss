#------------------------------------------------------------------------------
# RGBD integration + Spatial Input visualization.
# Press space to stop integrating. Then press space again to stop Spatial Input
# updates.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import open3d as o3d
import time
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_utilities
import hl2ss_mp
import hl2ss_3dcv
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

# Integration parameters
voxel_length = 0.5/100
sdf_trunc = 0.04
max_depth = 1.0

# Geometry parameters
head_frame_size = 0.1
eye_pointer_radius = 0.02
eye_pointer_color = np.array([0, 1, 0])
hand_joint_radius = 0.01
hand_joints_color = np.array([1, 0, 1])
hide_position = np.array([0, -20, 0])

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

    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)
    
    # Create Open3D integrator and visualizer ---------------------------------
    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    first_pcd = True

    # Create Spatial Input objects --------------------------------------------
    head_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=head_frame_size, origin=np.array([0.0, 0.0, 0.0]))
    head_previous_rotation = np.eye(3, 3)

    left_hand_joints = [o3d.geometry.TriangleMesh.create_octahedron(radius=hand_joint_radius) for _ in range(0, hl2ss.SI_HandJointKind.TOTAL)]
    right_hand_joints = [o3d.geometry.TriangleMesh.create_octahedron(radius=hand_joint_radius) for _ in range(0, hl2ss.SI_HandJointKind.TOTAL)]

    for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
        left_hand_joints[i].paint_uniform_color(hand_joints_color)
        right_hand_joints[i].paint_uniform_color(hand_joints_color)

    eye_pointer = o3d.geometry.TriangleMesh.create_octahedron(radius=eye_pointer_radius)
    eye_pointer.paint_uniform_color(eye_pointer_color)

    eye_line = o3d.geometry.LineSet()
    eye_line.colors = o3d.utility.Vector3dVector(np.array([0, 1, 0]).reshape((1, 3)))
    eye_line.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [0, 1, 0]]).reshape((2, 3)))
    eye_line.lines = o3d.utility.Vector2iVector(np.array([0, 1]).reshape((1,2)))

    # Start PV and RM Depth Long Throw streams --------------------------------    
    sink_pv = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, decoded_format='rgb24'))
    sink_depth = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))

    sink_pv.open()
    sink_depth.open()

    # Part 1: Integration -----------------------------------------------------
    pv_intrinsics = hl2ss_3dcv.pv_create_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)

    last_fs = -1

    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        _, fs_depth, data_depth = sink_depth.get_buffered_frame(-2)
        if ((data_depth is None) or (not hl2ss.is_valid_pose(data_depth.pose))):
            continue

        if (fs_depth <= last_fs):
            time.sleep(1 / hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)
            continue

        _, data_pv = sink_pv.get_nearest(data_depth.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        last_fs = fs_depth

        depth = hl2ss_3dcv.rm_depth_undistort(data_depth.payload.depth, calibration_lt.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = data_pv.payload.image

        pv_intrinsics = hl2ss_3dcv.pv_update_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
        color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)

        lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_depth.pose)
        world_to_lt       = hl2ss_3dcv.world_to_reference(data_depth.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
        world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics) @ hl2ss_3dcv.camera_to_image(color_intrinsics)
        world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
        pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
        color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR)

        mask_uv = hl2ss_3dcv.slice_to_block((pv_uv[:, :, 0] < 0) | (pv_uv[:, :, 0] >= pv_width) | (pv_uv[:, :, 1] < 0) | (pv_uv[:, :, 1] >= pv_height))
        depth[mask_uv] = 0

        color_image = o3d.geometry.Image(color)
        depth_image = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
        
        volume.integrate(rgbd, intrinsics_depth, world_to_lt.transpose())
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

    # Stop keyboard events ----------------------------------------------------
    listener.close()

    # Extract mesh and create ray casting scene -------------------------------
    triangles = volume.extract_triangle_mesh()
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(triangles)
    rs = o3d.t.geometry.RaycastingScene()
    rs.add_triangles(mesh)

    # Add head pose, hand joint, and gaze pointer geometry --------------------
    vis.add_geometry(head_frame, False)
    vis.add_geometry(eye_pointer, False)
    vis.add_geometry(eye_line, False)
    for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
        vis.add_geometry(left_hand_joints[i], False)
        vis.add_geometry(right_hand_joints[i], False)

    # Start keyboard events ---------------------------------------------------
    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()

    # Start Spatial Input stream ----------------------------------------------
    sink_si = hl2ss_mp.stream(hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT))
    sink_si.open()

    # Part 2: Draw Spatial Input objects in 3D --------------------------------
    while(not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        _, data_si = sink_si.get_most_recent_frame()
        if (data_si is None):
            continue

        si = data_si.payload

        if (si.head_pose_valid):
            head_pose = si.head_pose
            head_frame.translate(head_pose.position, False)
            head_rotation = hl2ss_3dcv.si_head_pose_rotation_matrix(head_pose.up, head_pose.forward)
            head_frame.rotate(head_rotation @ head_previous_rotation.transpose())
            head_previous_rotation = head_rotation
        else:
            head_frame.translate(hide_position, False)

        if (si.hand_left_valid):            
            left_hand = si.hand_left
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                left_hand_joints[i].translate(left_hand.position[i, :], False)
        else:
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                left_hand_joints[i].translate(hide_position, False)

        if (si.hand_right_valid):
            right_hand = si.hand_right
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                right_hand_joints[i].translate(right_hand.position[i, :], False)
        else:
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                right_hand_joints[i].translate(hide_position, False)

        updated_gaze = False
        if (si.eye_ray_valid):
            eye_ray = si.eye_ray
            ray = hl2ss_3dcv.si_ray_to_vector(eye_ray.origin, eye_ray.direction)
            rc = rs.cast_rays(ray)
            d = rc['t_hit'].numpy()
            if (np.isfinite(d)):
                eye_point = hl2ss_3dcv.si_ray_to_point(ray, d)
                eye_origin = hl2ss_3dcv.si_ray_get_origin(ray)
                eye_pointer.translate(eye_point.reshape((3, 1)), False)
                eye_line.points = o3d.utility.Vector3dVector(np.vstack((eye_point, eye_origin)))
                updated_gaze = True

        if (not updated_gaze):
            eye_pointer.translate(hide_position, False)
            eye_line.translate(hide_position, False)

        # Update visualizer ---------------------------------------------------
        vis.update_geometry(head_frame)
        vis.update_geometry(eye_pointer)
        vis.update_geometry(eye_line)
        for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
            vis.update_geometry(left_hand_joints[i])
            vis.update_geometry(right_hand_joints[i])

    # Stop Spatial Input stream -----------------------------------------------
    sink_si.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.close()

    # Show last Spatial Input status ------------------------------------------
    vis.run()
