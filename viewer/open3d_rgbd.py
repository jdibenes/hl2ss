#------------------------------------------------------------------------------
# This script demonstrates how to create aligned "rgbd" images, which can be
# used with Open3D, from the depth camera of the HoloLens.
#------------------------------------------------------------------------------

import numpy as np
import open3d as o3d
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_3dcv

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration folder for the depth camera
calibration_path = '../calibration/'

# Max depth in meters
max_depth = 3.0 

#------------------------------------------------------------------------------

# Get camera calibration

calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)

# Get single depth image

client = hl2ss.rx_decoded_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_0, hl2ss.PngFilterMode.Paeth)
client.open()
data_lt = client.get_next_packet()
client.close()

# Compute depth-to-depth registration constants

uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
xy1, scale, _ = hl2ss_3dcv.rm_depth_registration(uv2xy, calibration_lt.scale, calibration_lt.extrinsics, calibration_lt.intrinsics, calibration_lt.extrinsics)

# Generate RGBD pair

depth = hl2ss_3dcv.rm_depth_normalize(data_lt.payload.depth, calibration_lt.undistort_map, scale)
ab = cv2.remap(data_lt.payload.ab, calibration_lt.undistort_map[:, :, 0], calibration_lt.undistort_map[:, :, 1], cv2.INTER_LINEAR)
ab = hl2ss_3dcv.rm_depth_ab_to_uint8(ab / np.max(ab) * 0xFFFF) # AB scaled for visibility
rgb, depth = hl2ss_3dcv.rm_depth_rgbd(depth, ab)

# Show RGBD image

image = np.hstack((depth / max_depth, rgb / np.max(rgb))) # Depth and AB scaled for visibility
cv2.imshow('RGBD', image)
cv2.waitKey(0)

# Create Open3D RGBD Image

print(rgb.dtype)

o3d_rgb = o3d.geometry.Image(hl2ss_3dcv.rm_vlc_to_rgb(rgb))
o3d_depth = o3d.geometry.Image(depth)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
o3d.visualization.draw_geometries([pcd])
