#------------------------------------------------------------------------------
# This script captures a single depth image from the HoloLens and converts it
# to a pointcloud. 3D points are in meters.
#------------------------------------------------------------------------------

import hl2ss
import open3d as o3d
import numpy as np

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

#------------------------------------------------------------------------------

calibration = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
pixels = hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS

x = calibration.uv2xy[:, :, 0].reshape((pixels, 1))
y = calibration.uv2xy[:, :, 1].reshape((pixels, 1))
z = np.ones((pixels, 1))
n = np.sqrt(x**2 + y**2 + 1)
v = np.hstack((x, y, z)) / n

rx_depth = hl2ss.rx_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1)

rx_depth.open()
data = rx_depth.get_next_packet()
images = hl2ss.unpack_rm_depth(data.payload)
rx_depth.close()

d = images.depth.reshape((pixels, 1)) / calibration.scale
xyz = v * d
xyz = xyz[d.reshape((pixels,)) > 0] 

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.visualization.draw_geometries([pcd])
