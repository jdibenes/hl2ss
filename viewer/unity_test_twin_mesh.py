
import multiprocessing as mp
import numpy as np
import cv2
import hl2ss
import hl2ss_mp
import hl2ss_3dcv
import open3d as o3d
import rus

host = '192.168.1.7'
calibration_path = '../calibration'

width = 640
height = 360
framerate = 30
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = hl2ss.get_video_codec_bitrate(width, height, framerate, 1/142)
buffer_length = 10

# Integration parameters
voxel_length = 1/32
sdf_trunc = 0.04
max_depth = 4.0

if __name__ == '__main__':
    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)

    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])

    producer = hl2ss_mp.producer()
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate, 'rgb24')
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1, hl2ss.PngFilterMode.Paeth)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, framerate * buffer_length)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS * buffer_length)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_depth = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)
    sink_pv.get_attach_response()
    sink_depth.get_attach_response()

    pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()

    while (True):
        sink_depth.acquire()

        _, data_lt = sink_depth.get_most_recent_frame()
        if ((data_lt is None) or (not hl2ss.is_valid_pose(data_lt.pose))):
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        break

    sink_pv.detach()
    sink_depth.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    depth = hl2ss_3dcv.rm_depth_undistort(data_lt.payload.depth, calibration_lt.undistort_map)
    depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
    color = data_pv.payload.image
    pv_intrinsics = hl2ss.update_pv_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)

    lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
    lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose)
    world_to_lt       = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
    world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)
    world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
    pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
    color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR)

    mask_uv = hl2ss_3dcv.slice_to_block((pv_uv[:, :, 0] < 0) | (pv_uv[:, :, 0] >= width) | (pv_uv[:, :, 1] < 0) | (pv_uv[:, :, 1] >= height))
    depth[mask_uv] = 0

    color_image = o3d.geometry.Image(color)
    depth_image = o3d.geometry.Image(depth)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)

    depth_world_to_camera = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)

    volume.integrate(rgbd, intrinsics_depth, depth_world_to_camera.transpose())

    mesh = volume.extract_triangle_mesh()
    vertices = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    l_vertices = vertices @ np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]], dtype=np.float32)
    for i in range(0, tris.shape[0]):
        a = tris[i, 0]
        b = tris[i, 1]
        c = tris[i, 2]
        tris[i, 0] = c
        tris[i, 1] = b
        tris[i, 2] = a
    mesh.vertices = o3d.utility.Vector3dVector(l_vertices)
    mesh.triangles = o3d.utility.Vector3iVector(tris)

    print(f'Mesh triangles {tris.shape}')

    o3d.io.write_triangle_mesh('./data/mesh.obj', mesh)

    with open('./data/mesh.obj', mode='a') as file:
        file.write('# end of file')

    with open('./data/mesh.obj', mode='rb') as file:
        mesh_bytes = file.read()

    ipc_umq = hl2ss.ipc_umq(host, hl2ss.IPCPort.UNITY_MESSAGE_QUEUE)
    ipc_umq.open()

    display_list = rus.command_buffer()
    display_list.remove_all()
    display_list.load_mesh(mesh_bytes)

    ipc_umq.push(display_list)
    results = ipc_umq.pull(display_list)
    key = results[1]

    display_list = rus.command_buffer()
    display_list.set_world_transform(key, [0, 0, 0], [0, 0, 0, 1], [1, 1, 1])
    ipc_umq.push(display_list)
    ipc_umq.pull(display_list)

    ipc_umq.close()

    print(f'Created object with id {key}')

    o3d.visualization.draw_geometries([mesh])

