#------------------------------------------------------------------------------
# Sketch for pointcloud generation from HoloLens data.
#------------------------------------------------------------------------------

import multiprocessing as mp
import numpy as np
import hl2ss
import hl2ss_mp
import av
import open3d as o3d
import keyboard

# Settings --------------------------------------------------------------------

host = '192.168.1.15'
width = 640
height = 360
framerate = 30
profile = hl2ss.VideoProfile.H264_BASE
bitrate = 5*1024*1024
max_range = 2
voxel_size = 0.005

#------------------------------------------------------------------------------

if __name__ == '__main__':
    calibration_depth = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    calibration_pv = hl2ss.download_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width, height, framerate, profile, bitrate)

    pixels = hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS

    x = calibration_depth.uv2xy[:, :, 0].reshape((pixels, 1))
    y = calibration_depth.uv2xy[:, :, 1].reshape((pixels, 1))
    z = np.ones((pixels, 1))
    n = np.sqrt(x**2 + y**2 + 1)
    v = np.hstack((x, y, z)) / n

    #

    decoder = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')
    pcd_all = o3d.geometry.PointCloud()

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    #

    receiver_pv = hl2ss.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate)
    receiver_lt = hl2ss.rx_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1)

    producer_pv = hl2ss_mp.producer(receiver_pv, framerate * 30)
    producer_lt = hl2ss_mp.producer(receiver_lt, hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS * 30)

    producer_lt.start()
    producer_pv.start()

    manager = mp.Manager()

    buffer_pv = hl2ss_mp.RingBuffer(framerate * 30)

    sink_pv = producer_pv.create_sink(hl2ss_mp.get_sync_slot_pv(framerate), manager, None)
    sink_lt = producer_lt.create_sink(hl2ss_mp.get_sync_slot_rm_depth(), manager, sink_pv)

    sink_pv.get_attach_response()
    sink_lt.get_attach_response()

    sink_pv.wait_for_sync()
    sink_lt.wait_for_sync()

    while (not keyboard.is_pressed('space')):
        sink_pv.target_stamp = sink_pv.get_frame_stamp() + 1
        sink_lt.target_stamp = sink_lt.get_frame_stamp() + 1

        for frame_stamp in range(sink_pv.frame_stamp, sink_pv.target_stamp, 1):
            _, data_pv = sink_pv.get_buffered_frame(frame_stamp)
            for packet in decoder.parse(data_pv.payload):
                for frame in decoder.decode(packet):
                    data_pv.payload = frame
                    buffer_pv.append(data_pv)
        sink_pv.frame_stamp = sink_pv.target_stamp

        data_lt = None
        for frame_stamp in range(sink_lt.frame_stamp, sink_lt.target_stamp, 1):
            _, data_lt = sink_lt.get_buffered_frame(frame_stamp)
        sink_lt.frame_stamp = sink_lt.target_stamp
        if (data_lt is None):
            continue
        if (data_lt.pose[3,3] == 0.0):
            continue

    #

        data_pv = None
        min_ds = 0xFFFFFFFFFFFFFFFF
        list_pv = buffer_pv.get()
        for i in range(buffer_pv.length(), 0, -1):
            data = list_pv[i - 1]
            ds = np.abs(data.timestamp - data_lt.timestamp)
            if (ds < min_ds):
                min_ds = ds
                data_pv = data
            else:
                break
        if (data_pv is None):
            continue
        if (data_pv.pose[3,3] == 0.0):
            continue

    #

        image_pv = data_pv.payload.to_ndarray(format='rgb24')
        image_lt = hl2ss.unpack_rm_depth(data_lt.payload)

        d = image_lt.depth.reshape((pixels, 1)) / calibration_depth.scale
        xyz = v * d
        xyz = xyz[(d.reshape((pixels,)) > 0) & (d.reshape((pixels,)) < max_range)]
        if (xyz.shape[0] <= 0):
            continue

        xyz1 = np.hstack((xyz, np.ones((xyz.shape[0], 1)))) @ np.linalg.inv(calibration_depth.extrinsics) @ data_lt.pose
        xyz1_pv = xyz1 @ np.linalg.inv(data_pv.pose) @ calibration_pv.projection
        xyz_pv = xyz1_pv[:, 0:3] / xyz1_pv[:, 2].reshape((xyz.shape[0], 1))
        uv_pv = xyz_pv[:, 0:2].astype(int)

        select = (uv_pv[:, 0] >= 0) & (uv_pv[:, 0] < width) & (uv_pv[:, 1] >= 0) & (uv_pv[:, 1] < height)
        uv_pv = uv_pv[select,:]
        xyz = xyz1[select, 0:3]
        if (xyz.shape[0] <= 0):
            continue

        colors = [image_pv[uv[1], uv[0]] for uv in uv_pv]
        colors = np.stack(colors, axis=0).astype(float) / 255.0

    #

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        pcd_all += pcd

        vis.add_geometry(pcd) 
        vis.poll_events()
        vis.update_renderer()

    #

        sink_pv.wait_for_data()

    pcd_all = pcd_all.voxel_down_sample(voxel_size=voxel_size)
    #o3d.io.write_point_cloud("test.ply", pcd_all)

    sink_pv.detach()
    sink_lt.detach()

    producer_pv.stop()
    producer_lt.stop()

    vis.destroy_window()
    
    o3d.visualization.draw_geometries([pcd_all])
