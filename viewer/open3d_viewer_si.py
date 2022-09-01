#------------------------------------------------------------------------------
# RGBD integration + Spatial Input visualization.
#------------------------------------------------------------------------------

import multiprocessing as mp
import numpy as np
import open3d as o3d
import keyboard
import hl2ss
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Camera parameters
width = 640
height = 360
framerate = 30
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = 5*1024*1024
focus = 1000
exposure = hl2ss.ExposureValue.Max // 2
white_balance = hl2ss.ColorTemperaturePreset.Manual
model_depth_path = '../calibration/rm_depth_longthrow'
model_pv_path = '../calibration/pv'

# Buffer length in seconds
buffer_length = 10

# Integration parameters
voxel_length = 2/100
sdf_trunc = 0.04
max_depth = 3.0

eye_pointer_color = np.array([0, 1, 0])
hand_joints_color = np.array([1, 0, 1])
hide_position = np.array([0, -20, 0], dtype=np.float32)

#------------------------------------------------------------------------------

if __name__ == '__main__':
    hl2ss_utilities.pv_optimize_for_cv(host, focus, exposure, white_balance)

    calibration_pv = hl2ss.download_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width, height, framerate, profile, bitrate)
    extrinsics_pv = hl2ss_utilities.pv_load_rm_extrinsics(model_pv_path)
    model_depth = hl2ss_utilities.rm_depth_longthrow_load_pinhole_model(model_depth_path)
    depth_scale = hl2ss_utilities.rm_depth_get_normalizer(model_depth.uv2xy)
    
    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, model_depth.intrinsics[0, 0], model_depth.intrinsics[1, 1], model_depth.intrinsics[2, 0], model_depth.intrinsics[2, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    first_pcd = True

    head_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=np.array([0.0, 0.0, 0.0]))
    head_previous_rotation = np.eye(3, 3)
    eye_octo = o3d.geometry.TriangleMesh.create_octahedron(radius=0.02)
    left_hand_octo = [o3d.geometry.TriangleMesh.create_octahedron(radius=0.01) for _ in range(0, hl2ss.HandJointKind.TOTAL)]
    right_hand_octo = [o3d.geometry.TriangleMesh.create_octahedron(radius=0.01) for _ in range(0, hl2ss.HandJointKind.TOTAL)]
    
    eye_octo.paint_uniform_color(eye_pointer_color)
    vis.add_geometry(head_frame, False)
    vis.add_geometry(eye_octo, False)    
    for i in range(0, hl2ss.HandJointKind.TOTAL):
        left_hand_octo[i].paint_uniform_color(hand_joints_color)
        right_hand_octo[i].paint_uniform_color(hand_joints_color)
        vis.add_geometry(left_hand_octo[i], False)
        vis.add_geometry(right_hand_octo[i], False)

    producer = hl2ss_utilities.producer()
    producer.initialize_decoded_pv(framerate * buffer_length, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_0, width, height, framerate, profile, bitrate, 'rgb24')
    producer.initialize_decoded_rm_depth(hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS * buffer_length, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1)
    producer.initialize_si(hl2ss.Parameters_SI.SAMPLE_RATE * buffer_length, host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)
    producer.start()

    manager = mp.Manager()
    consumer = hl2ss_utilities.consumer()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_depth = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)
    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, manager, None)

    sinks = [sink_pv, sink_depth, sink_si]

    [sink.get_attach_response() for sink in sinks]

    while (not keyboard.is_pressed('space')):
        sink_depth.acquire()

        data_depth = sink_depth.get_most_recent_frame()
        if (not data_depth.is_valid_pose()):
            continue

        _, data_pv = sink_pv.get_nearest(data_depth.timestamp)
        if (data_pv is None):
            continue

        _, data_si = sink_si.get_nearest(data_depth.timestamp)
        if (data_si is None):
            continue

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
            triangles = volume.extract_triangle_mesh()
            if (not triangles.is_empty()):
                mesh = o3d.t.geometry.TriangleMesh.from_legacy(triangles)
                rs = o3d.t.geometry.RaycastingScene()
                rs.add_triangles(mesh)
                rc = rs.cast_rays(ray)
                d = rc['t_hit'].numpy()
                if (not np.isinf(d)):
                    eye_octo.translate(ray[0, 0:3] + d*ray[0, 3:6], False)
                else:
                    eye_octo.translate(hide_position, False)
            else:
                eye_octo.translate(hide_position, False)
        else:
            eye_octo.translate(hide_position, False)

        if (si.is_valid_hand_left()):            
            left_hand = hl2ss_utilities.si_unpack_hand(si.get_hand_left())
            for i in range(0, hl2ss.HandJointKind.TOTAL):
                left_hand_octo[i].translate(left_hand.positions[i, :], False)
        else:
            for i in range(0, hl2ss.HandJointKind.TOTAL):
                left_hand_octo[i].translate(hide_position, False)

        if (si.is_valid_hand_right()):
            right_hand = hl2ss_utilities.si_unpack_hand(si.get_hand_right())
            for i in range(0, hl2ss.HandJointKind.TOTAL):
                right_hand_octo[i].translate(right_hand.positions[i, :], False)
        else:
            for i in range(0, hl2ss.HandJointKind.TOTAL):
                right_hand_octo[i].translate(hide_position, False)

        vis.update_geometry(head_frame)
        vis.update_geometry(eye_octo)
        for i in range(0, hl2ss.HandJointKind.TOTAL):
            vis.update_geometry(left_hand_octo[i])
            vis.update_geometry(right_hand_octo[i])

        hands = si.is_valid_hand_left() or si.is_valid_hand_right()

        if (not hands):
            depth_world_to_camera = hl2ss_utilities.rm_world_to_camera(model_depth.extrinsics, data_depth.pose)
            rgb, depth = hl2ss_utilities.rm_depth_generate_rgbd_from_pv(data_pv.payload, data_depth.payload.depth, calibration_pv.intrinsics, extrinsics_pv, model_depth.map, depth_scale, model_depth.uv2xy, model_depth.extrinsics)
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

    [sink.detach() for sink in sinks]
    producer.stop()

    vis.run()
