#------------------------------------------------------------------------------
# This script demonstrates how to create aligned "rgbd" images, which can be
# used with Open3D, from the depth camera of the HoloLens.
#------------------------------------------------------------------------------

import numpy as np
import open3d as o3d
import cv2
import hl2ss
import hl2ss_utilities

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Calibration folder for the depth camera
model_depth_path = '../calibration/rm_depth_longthrow'

# Max depth in meters
max_depth = 3.0 

#------------------------------------------------------------------------------

calibration_depth = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
model_depth = hl2ss_utilities.rm_depth_longthrow_load_pinhole_model(model_depth_path)
depth_scale = hl2ss_utilities.rm_depth_get_normalizer(model_depth.uv2xy, calibration_depth.scale)

client = hl2ss_utilities.rx_decoded_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1)
client.open()
data_depth = client.get_next_packet()
client.close()

rgb, depth = hl2ss_utilities.rm_depth_generate_rgbd(data_depth.payload.ab, data_depth.payload.depth, model_depth.map, depth_scale)
world_to_camera_depth = hl2ss_utilities.rm_world_to_camera(calibration_depth.extrinsics, data_depth.pose)

o3d_rgb = o3d.geometry.Image(np.dstack((rgb, rgb, rgb)))
o3d_depth = o3d.geometry.Image(depth)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, model_depth.intrinsics[0, 0], model_depth.intrinsics[1, 1], model_depth.intrinsics[2, 0], model_depth.intrinsics[2, 1])

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics, world_to_camera_depth.transpose())
o3d.visualization.draw_geometries([pcd])

cv2.imshow('rgb', rgb)
cv2.imshow('depth', depth / np.max(depth))
cv2.waitKey(0)
