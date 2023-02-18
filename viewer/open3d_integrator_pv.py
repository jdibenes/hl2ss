#------------------------------------------------------------------------------
# RGBD integration using Open3D. Color information comes from the front RGB
# camera. Press space to stop.
#------------------------------------------------------------------------------
# TODO: USE CAMERA POSES FOR MORE ACCURATE REGISTRATION

from pynput import keyboard

import multiprocessing as mp
import open3d as o3d
import cv2
import hl2ss
import hl2ss_mp
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

calibration_path = '../calibration'

# Camera parameters
focus = 1000
width = 640
height = 360
framerate = 30
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = 5*1024*1024
exposure_mode = hl2ss.PV_ExposureMode.Manual
exposure = hl2ss.PV_ExposureValue.Max // 4
iso_speed_mode = hl2ss.PV_IsoSpeedMode.Manual
iso_speed_value = 1600
white_balance = hl2ss.PV_ColorTemperaturePreset.Manual

# Buffer length in seconds
buffer_length = 10

# Integration parameters
voxel_length = 1/100
sdf_trunc = 0.04
max_depth = 3.0

#------------------------------------------------------------------------------

if __name__ == '__main__':
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
    rc_control = hl2ss.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    while (not rc_control.get_pv_subsystem_status()):
        pass

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

    sinks = [sink_pv, sink_depth]

    [sink.get_attach_response() for sink in sinks]
 
    while (enable):
        sink_depth.acquire()

        data_lt = sink_depth.get_most_recent_frame()
        if (not hl2ss.is_valid_pose(data_lt.pose)):
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        depth = hl2ss_3dcv.rm_depth_normalize(data_lt.payload.depth, calibration_lt.undistort_map, scale)
        depth_to_pv_image = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose) @ hl2ss_3dcv.world_to_reference(data_pv.pose) @ calibration_pv.intrinsics

        rgb, depth = hl2ss_3dcv.rm_depth_rgbd_registered(depth, data_pv.payload.image, xy1, depth_to_pv_image, cv2.INTER_LINEAR)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb), o3d.geometry.Image(depth), depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
        depth_world_to_camera = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)

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
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    listener.join()

    vis.run()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
