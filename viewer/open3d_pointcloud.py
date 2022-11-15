#------------------------------------------------------------------------------
# This script captures a single depth image from the HoloLens and converts it
# to a pointcloud. 3D points are in meters.
#------------------------------------------------------------------------------

import open3d as o3d
import hl2ss
import hl2ss_3dcv
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Max depth in meters
max_depth = 3.0

#------------------------------------------------------------------------------

# Get camera calibration

calibration = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

# Compute depth-to-rgb registration constants

xy1, scale, _ = hl2ss_3dcv.rm_depth_registration(calibration.uv2xy, calibration.scale, calibration.extrinsics, calibration.extrinsics, calibration.extrinsics)

# Get single depth image

rx_depth = hl2ss.rx_decoded_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_0, hl2ss.PngFilterMode.Paeth)
rx_depth.open()
data = rx_depth.get_next_packet()
rx_depth.close()

# Convert depth to 3D points

depth = hl2ss_3dcv.rm_depth_scale(data.payload.depth, scale)

xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
xyz = xyz[(xyz[:, 2] > 0) & (xyz[:, 2] < max_depth), :] 

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)

o3d.visualization.draw_geometries([pcd])
