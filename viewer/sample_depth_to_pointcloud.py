#------------------------------------------------------------------------------
# This script captures a single depth image from the HoloLens and converts it
# to a pointcloud. 3D points are in meters.
#------------------------------------------------------------------------------

import open3d as o3d
import hl2ss
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration folder (must exist but can be empty)
calibration_path = '../calibration'

# Max depth in meters
max_depth = 3.0

#------------------------------------------------------------------------------

# Get camera calibration ------------------------------------------------------
# Calibration data will be downloaded if it's not in the calibration folder
calibration = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(calibration.uv2xy, calibration.scale)

# Get single depth image ------------------------------------------------------
rx_depth = hl2ss.rx_decoded_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_0, hl2ss.PngFilterMode.Paeth)
rx_depth.open()
data = rx_depth.get_next_packet()
rx_depth.close()

# Convert depth to 3D points --------------------------------------------------
depth = hl2ss_3dcv.rm_depth_normalize(data.payload.depth, scale)
xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
xyz = hl2ss_3dcv.block_to_list(xyz)
d = hl2ss_3dcv.block_to_list(depth).reshape((-1,))
xyz = xyz[(d > 0) & (d < max_depth), :]

# Visualize pointcloud --------------------------------------------------------
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.visualization.draw_geometries([pcd])
