#------------------------------------------------------------------------------
# RGBD integration + Spatial Input visualization.
# Press space to stop integrating. Then press space again to stop Spatial Input
# updates.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import open3d as o3d
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_utilities
import hl2ss_mp
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

# Camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 30

# Buffer length in seconds
buffer_length = 10

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
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, decoded_format='rgb24'))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, pv_framerate * buffer_length)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS * buffer_length)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_depth = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)
    
    sink_pv.get_attach_response()
    sink_depth.get_attach_response()

    # Part 1: Integration -----------------------------------------------------
    pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)

    while (enable):
        sink_depth.acquire()

        _, data_depth = sink_depth.get_most_recent_frame()
        if ((data_depth is None) or (not hl2ss.is_valid_pose(data_depth.pose))):
            continue

        _, data_pv = sink_pv.get_nearest(data_depth.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        depth = hl2ss_3dcv.rm_depth_undistort(data_depth.payload.depth, calibration_lt.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        color = data_pv.payload.image

        pv_intrinsics = hl2ss.update_pv_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
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

        vis.poll_events()
        vis.update_renderer()

    # Stop PV and RM Depth Long Throw streams ---------------------------------
    sink_pv.detach()
    sink_depth.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

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
    enable = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Start Spatial Input stream ----------------------------------------------
    producer.configure(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT))
    producer.initialize(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(hl2ss.StreamPort.SPATIAL_INPUT)

    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, manager, ...)
    sink_si.get_attach_response()

    # Part 2: Draw Spatial Input objects in 3D --------------------------------
    while(enable):
        sink_si.acquire()

        _, data_si = sink_si.get_most_recent_frame()
        if (data_si is None):
            continue

        si = hl2ss.unpack_si(data_si.payload)

        if (si.is_valid_head_pose()):
            head_pose = si.get_head_pose()
            head_frame.translate(head_pose.position, False)
            head_rotation = hl2ss_utilities.si_head_pose_rotation_matrix(head_pose.up, head_pose.forward)
            head_frame.rotate(head_rotation @ head_previous_rotation.transpose())
            head_previous_rotation = head_rotation
        else:
            head_frame.translate(hide_position, False)

        if (si.is_valid_hand_left()):            
            left_hand = hl2ss_utilities.si_unpack_hand(si.get_hand_left())
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                left_hand_joints[i].translate(left_hand.positions[i, :], False)
        else:
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                left_hand_joints[i].translate(hide_position, False)

        if (si.is_valid_hand_right()):
            right_hand = hl2ss_utilities.si_unpack_hand(si.get_hand_right())
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                right_hand_joints[i].translate(right_hand.positions[i, :], False)
        else:
            for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
                right_hand_joints[i].translate(hide_position, False)

        updated_gaze = False
        if (si.is_valid_eye_ray()):
            eye_ray = si.get_eye_ray()
            ray = hl2ss_utilities.si_ray_to_vector(eye_ray.origin, eye_ray.direction)
            rc = rs.cast_rays(ray)
            d = rc['t_hit'].numpy()
            if (np.isfinite(d)):
                eye_point = hl2ss_utilities.si_ray_to_point(ray, d)
                eye_origin = hl2ss_utilities.si_ray_get_origin(ray)
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

        vis.poll_events()
        vis.update_renderer()

    # Stop Spatial Input stream -----------------------------------------------
    sink_si.detach()
    producer.stop(hl2ss.StreamPort.SPATIAL_INPUT)

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

    # Show last Spatial Input status ------------------------------------------
    vis.run()
