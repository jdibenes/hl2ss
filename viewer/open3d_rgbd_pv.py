#------------------------------------------------------------------------------
# This script demonstrates how to create aligned rgbd images, which can be used
# with Open3D, from the depth and front RGB cameras of the HoloLens.
#------------------------------------------------------------------------------

import numpy as np
import open3d as o3d
import hl2ss
import hl2ss_utilities
import cv2

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Front RGB camera parameters
pv_width = 640
pv_height = 360
pv_framerate = 30

# Video encoding profile
pv_profile = hl2ss.VideoProfile.H264_BASE

# Encoded stream average bits per second
# Must be > 0
pv_bitrate = 1*1024*1024

# Focus in mm for the front RGB camera
pv_focus = 1000

# Calibration folder for the front camera
model_pv_path = '../calibration/pv'

# Calibration folder for the depth camera
model_depth_path = '../calibration/rm_depth_longthrow'

# Max depth in meters
max_depth = 3.0 

#------------------------------------------------------------------------------

settings = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
settings.set_focus(hl2ss.FocusMode.Manual, hl2ss.AutoFocusRange.Normal, hl2ss.ManualFocusDistance.Infinity, pv_focus, hl2ss.DriverFallback.Disable)

calibration_pv = hl2ss.download_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate)
pv_extrinsics = hl2ss_utilities.pv_load_rm_extrinsics(model_pv_path)
model_depth = hl2ss_utilities.rm_depth_longthrow_load_pinhole_model(model_depth_path)
depth_scale = hl2ss_utilities.rm_depth_get_normalizer(model_depth.uv2xy)

client = hl2ss_utilities.rx_decoded_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1)
client.open()
data_depth = client.get_next_packet()
client.close()

client = hl2ss_utilities.rx_decoded_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_0, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate, 'rgb24')
client.open()
data_pv = client.get_next_packet()
client.close()

rgb, depth = hl2ss_utilities.rm_depth_generate_rgbd_from_pv(data_pv.payload, data_depth.payload.depth, calibration_pv.intrinsics, pv_extrinsics, model_depth.map, depth_scale, model_depth.uv2xy, model_depth.extrinsics)
world_to_camera_depth = hl2ss_utilities.rm_world_to_camera(model_depth.extrinsics, data_depth.pose)

o3d_rgb = o3d.geometry.Image(rgb)
o3d_depth = o3d.geometry.Image(depth)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, model_depth.intrinsics[0, 0], model_depth.intrinsics[1, 1], model_depth.intrinsics[2, 0], model_depth.intrinsics[2, 1])

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics, world_to_camera_depth.transpose())
o3d.visualization.draw_geometries([pcd])

cv2.imshow('rgb', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
cv2.imshow('depth', depth / np.max(depth))
cv2.waitKey(0)
