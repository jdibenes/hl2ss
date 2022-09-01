#------------------------------------------------------------------------------
# This script captures a single depth image from the HoloLens and converts it
# to a pointcloud. 3D points are in meters.
#------------------------------------------------------------------------------

import open3d as o3d
import hl2ss
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Max depth in meters
max_depth = 3.0

#------------------------------------------------------------------------------

calibration = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
rx_depth = hl2ss_utilities.rx_decoded_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_0)
depth_scale = hl2ss_utilities.rm_depth_get_normalizer(calibration.uv2xy)

rx_depth.open()
data = rx_depth.get_next_packet()
rx_depth.close()

depth = data.payload.depth / depth_scale
xyz = hl2ss_utilities.rm_depth_to_points(depth, calibration.uv2xy)
xyz = xyz[(xyz[:, 2] > 0) & (xyz[:, 2] < max_depth), :] 

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)

o3d.visualization.draw_geometries([pcd])
