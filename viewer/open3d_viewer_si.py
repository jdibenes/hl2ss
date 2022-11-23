#------------------------------------------------------------------------------
# RGBD integration + Spatial Input visualization. Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import numpy as np
import open3d as o3d
import cv2
import hl2ss
import hl2ss_utilities
import hl2ss_mp
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'
calibration_path = '../calibration'

# Camera parameters
width = 1280
height = 720
framerate = 30
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = 5*1024*1024
focus = 1000
exposure_mode = hl2ss.PV_ExposureMode.Manual
exposure = hl2ss.PV_ExposureValue.Max // 2
iso_speed_mode = hl2ss.PV_IsoSpeedMode.Manual
iso_speed_value = 1600
white_balance = hl2ss.PV_ColorTemperaturePreset.Manual


# Buffer length in seconds
buffer_length = 10

# Integration parameters
voxel_length = 0.25/100
sdf_trunc = 0.04
max_depth = 1.0

# Geometry parameters
head_frame_size = 0.1
eye_pointer_radius = 0.02
eye_pointer_color = np.array([0, 1, 0])
hand_joint_radius = 0.01
hand_joints_color = np.array([1, 0, 1])
hide_position = np.array([0, -20, 0], dtype=np.float32)

#------------------------------------------------------------------------------

if __name__ == '__main__':
    enable = True

    def on_press(key):
        global enable
        if (key == keyboard.Key.space):
            enable = False
        print('key pressed')
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
    hl2ss_3dcv.pv_optimize_for_cv(host, focus, exposure_mode, exposure, iso_speed_mode, iso_speed_value, white_balance)

    calibration_pv = hl2ss_3dcv.get_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, calibration_path, focus, width, height, framerate, True)
    calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)

    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale, depth_to_pv_image = hl2ss_3dcv.rm_depth_registration(uv2xy, calibration_lt.scale, calibration_lt.extrinsics, calibration_pv.intrinsics, calibration_pv.extrinsics)
    
    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    first_pcd = True

    head_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=head_frame_size, origin=np.array([0.0, 0.0, 0.0]))
    head_previous_rotation = np.eye(3, 3)

    eye_pointer = o3d.geometry.TriangleMesh.create_octahedron(radius=eye_pointer_radius)
    eye_pointer.paint_uniform_color(eye_pointer_color)

    eye_line = o3d.geometry.LineSet()
    eye_line.colors = o3d.utility.Vector3dVector(np.array([0, 1, 0]).reshape((1, 3)))
    eye_line.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [0, 1, 0]]).reshape((2, 3)))
    eye_line.lines = o3d.utility.Vector2iVector(np.array([0, 1]).reshape((1,2)))

    left_hand_joints = [o3d.geometry.TriangleMesh.create_octahedron(radius=hand_joint_radius) for _ in range(0, hl2ss.SI_HandJointKind.TOTAL)]
    right_hand_joints = [o3d.geometry.TriangleMesh.create_octahedron(radius=hand_joint_radius) for _ in range(0, hl2ss.SI_HandJointKind.TOTAL)]

    for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
        left_hand_joints[i].paint_uniform_color(hand_joints_color)
        right_hand_joints[i].paint_uniform_color(hand_joints_color)

    producer = hl2ss_mp.producer()
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_0, width, height, framerate, profile, bitrate, 'rgb24')
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1, hl2ss.PngFilterMode.Paeth)
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, framerate * buffer_length)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS * buffer_length)
    producer.initialize(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    producer.start(hl2ss.StreamPort.SPATIAL_INPUT)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_depth = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)
    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, manager, ...)

    sinks = [sink_pv, sink_depth, sink_si]

    [sink.get_attach_response() for sink in sinks]

    while (enable):
        sink_depth.acquire()

        data_depth = sink_depth.get_most_recent_frame()
        if (not hl2ss.is_valid_pose(data_depth.pose)):
            continue

        _, data_pv = sink_pv.get_nearest(data_depth.timestamp)
        if (data_pv is None):
            continue

        depth_world_to_camera = hl2ss_3dcv.world_to_reference(data_depth.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)

        depth = hl2ss_3dcv.rm_depth_normalize(data_depth.payload.depth, calibration_lt.undistort_map, scale)
        rgb, depth = hl2ss_3dcv.rm_depth_rgbd_registered(depth, data_pv.payload, xy1, depth_to_pv_image, cv2.INTER_LINEAR)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb), o3d.geometry.Image(depth), depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
        
        volume.integrate(rgbd, intrinsics_depth, depth_world_to_camera.transpose())
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

    listener.stop()

    triangles = volume.extract_triangle_mesh()
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(triangles)
    rs = o3d.t.geometry.RaycastingScene()
    rs.add_triangles(mesh)

    vis.add_geometry(head_frame, False)
    vis.add_geometry(eye_pointer, False)
    vis.add_geometry(eye_line, False)
    for i in range(0, hl2ss.SI_HandJointKind.TOTAL):        
        vis.add_geometry(left_hand_joints[i], False)
        vis.add_geometry(right_hand_joints[i], False)

    enable = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while(enable):
        sink_si.acquire()

        data_si = sink_si.get_most_recent_frame()

        si = hl2ss.unpack_si(data_si.payload)

        if (si.is_valid_head_pose()):
            head_pose = si.get_head_pose()
            head_frame.translate(head_pose.position, False)
            head_rotation = hl2ss_utilities.si_head_pose_rotation_matrix(head_pose)
            head_frame.rotate(head_rotation @ head_previous_rotation.transpose())
            head_previous_rotation = head_rotation
        else:
            head_frame.translate(hide_position, False)
        
        if (si.is_valid_eye_ray()):
            eye_ray = si.get_eye_ray()
            ray = np.hstack((eye_ray.origin.reshape((1, -1)), eye_ray.direction.reshape((1,-1))))

            rc = rs.cast_rays(ray)
            d = rc['t_hit'].numpy()
            if (not np.isinf(d)):
                eye_point = ray[0, 0:3] + d*ray[0, 3:6]
                eye_pointer.translate(eye_point, False)
                eye_line.points = o3d.utility.Vector3dVector(np.array([eye_point, ray[0, 0:3]]).reshape((2, 3)))
            else:
                eye_pointer.translate(hide_position, False)
                eye_line.translate(hide_position, False)
        else:
            eye_pointer.translate(hide_position, False)
            eye_line.translate(hide_position, False)

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

        vis.update_geometry(head_frame)
        vis.update_geometry(eye_pointer)
        vis.update_geometry(eye_line)
        for i in range(0, hl2ss.SI_HandJointKind.TOTAL):
            vis.update_geometry(left_hand_joints[i])
            vis.update_geometry(right_hand_joints[i])

        vis.poll_events()
        vis.update_renderer()

    [sink.detach() for sink in sinks]
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    producer.stop(hl2ss.StreamPort.SPATIAL_INPUT)
    listener.join()

    vis.run()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
