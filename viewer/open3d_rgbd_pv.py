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
rc_control = hl2ss.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
while (not rc_control.get_pv_subsystem_status()):
    pass

# Fix PV focus

hl2ss_3dcv.pv_optimize_for_cv(host, pv_focus, hl2ss.PV_ExposureMode.Auto, hl2ss.PV_ExposureValue.Min, hl2ss.PV_IsoSpeedMode.Auto, hl2ss.PV_IsoSpeedValue.Max, hl2ss.PV_ColorTemperaturePreset.Auto)

# Get camera calibrations

pv_calibration = hl2ss_3dcv.get_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, calibration_path, pv_focus, pv_width, pv_height, pv_framerate, True)
pv_calibration.intrinsics, pv_calibration.extrinsics, _ = hl2ss_3dcv.pv_fix_calibration(pv_calibration.intrinsics, pv_calibration.extrinsics)
lt_calibration = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)

# Get single depth and pv images
# To keep this example simple, the images are captured one after the other but this will not work properly for dynamic scenes
# Use the multiprocessing producer to obtain image pairs that are closest in time

client = hl2ss.rx_decoded_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_0, hl2ss.PngFilterMode.Paeth)
client.open()
data_lt = client.get_next_packet()
client.close()

client = hl2ss.rx_decoded_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_0, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate, 'rgb24')
client.open()
data_pv = client.get_next_packet()
client.close()

# Compute depth-to-rgb registration constants

uv2xy = hl2ss_3dcv.compute_uv2xy(lt_calibration.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
xy1, scale, depth_to_pv_image = hl2ss_3dcv.rm_depth_registration(uv2xy, lt_calibration.scale, lt_calibration.extrinsics, pv_calibration.intrinsics, pv_calibration.extrinsics)

# Generate RGBD pair

depth = hl2ss_3dcv.rm_depth_normalize(data_lt.payload.depth, lt_calibration.undistort_map, scale)
rgb, depth = hl2ss_3dcv.rm_depth_rgbd_registered(depth, data_pv.payload.image, xy1, depth_to_pv_image, cv2.INTER_LINEAR)

# Show RGBD image

image = np.hstack((hl2ss_3dcv.rm_vlc_to_rgb(depth) / max_depth, rgb / 255)) # Depth scaled for visibility
cv2.imshow('RGBD', image)
cv2.waitKey(0)

# Create Open3D RGBD Image

o3d_rgb = o3d.geometry.Image(rgb)
o3d_depth = o3d.geometry.Image(depth)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth, depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
intrinsics = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, lt_calibration.intrinsics[0, 0], lt_calibration.intrinsics[1, 1], lt_calibration.intrinsics[2, 0], lt_calibration.intrinsics[2, 1])

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
o3d.visualization.draw_geometries([pcd])

hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
