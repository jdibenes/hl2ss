#------------------------------------------------------------------------------
# This script demonstrates how to create aligned rgbd images, which can be used
# with Open3D, from the depth and front RGB cameras of the HoloLens.
#------------------------------------------------------------------------------

import numpy as np
import open3d as o3d
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_utilities
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Front RGB camera parameters
pv_focus = 1000
pv_width = 640
pv_height = 360
pv_framerate = 30

# Video encoding profile
pv_profile = hl2ss.VideoProfile.H264_BASE

# Encoded stream average bits per second
# Must be > 0
pv_bitrate = 1*1024*1024

# Calibration folder
calibration_path = '../calibration'

# Max depth in meters
max_depth = 3.0 

#------------------------------------------------------------------------------

hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

# Get camera calibrations

calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)

# Get single depth and pv images
# To keep this example simple, the images are captured one after the other but this will not work properly for dynamic scenes
# Use the multiprocessing producer to obtain image pairs that are closest in time

client = hl2ss.rx_decoded_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1, hl2ss.PngFilterMode.Paeth)
client.open()
data_lt = client.get_next_packet()
client.close()

client = hl2ss.rx_decoded_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate, 'rgb24')
client.open()
data_pv = client.get_next_packet()
client.close()

# Compute depth-to-rgb registration constants

uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

# Generate RGBD pair

depth = hl2ss_3dcv.rm_depth_undistort(data_lt.payload.depth, calibration_lt.undistort_map)
depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
color = data_pv.payload.image
pv_intrinsics = hl2ss.create_pv_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)

lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose)
world_to_lt       = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)
world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.camera_to_image(pv_intrinsics)
world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR)

mask_uv = hl2ss_3dcv.slice_to_block((pv_uv[:, :, 0] < 0) | (pv_uv[:, :, 0] >= pv_width) | (pv_uv[:, :, 1] < 0) | (pv_uv[:, :, 1] >= pv_height))
depth[mask_uv] = 0

# Show RGBD image

image = np.hstack((hl2ss_3dcv.rm_depth_to_rgb(depth) / max_depth, color / 255)) # Depth scaled for visibility
cv2.imshow('RGBD', image)
cv2.waitKey(0)

# Create Open3D RGBD Image

o3d_rgb = o3d.geometry.Image(color)
o3d_depth = o3d.geometry.Image(depth)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
o3d.visualization.draw_geometries([pcd])

hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
