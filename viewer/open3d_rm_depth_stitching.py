#------------------------------------------------------------------------------
# This script continuously captures depth images from the HoloLens and adds
# them to a pointcloud. Press Ctrl+C to stop.
#------------------------------------------------------------------------------

import hl2ss
import open3d as o3d
import numpy as np

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Voxel size
voxel_size = 0.001

#------------------------------------------------------------------------------

calibration = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
pixels = hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS

x = calibration.uv2xy[:, :, 0].reshape((pixels, 1))
y = calibration.uv2xy[:, :, 1].reshape((pixels, 1))
z = np.ones((pixels, 1))
n = np.sqrt(x**2 + y**2 + 1)
v = np.hstack((x, y, z)) / n

pcd_all = o3d.geometry.PointCloud()
vis = o3d.visualization.Visualizer()
vis.create_window()

frames = 0
rx_depth = hl2ss.rx_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1)
rx_depth.open()

try:
    while (True):
        data = rx_depth.get_next_packet()
        images = hl2ss.unpack_rm_depth(data.payload)
        
        d = images.depth.reshape((pixels, 1)) / calibration.scale
        xyz1 = np.hstack((v * d, np.ones((pixels, 1))))
        camera2world = np.linalg.inv(calibration.extrinsics) @ data.pose
        xyz1 = xyz1[d.reshape((pixels,)) > 0] @ camera2world

        pcd_next = o3d.geometry.PointCloud()
        pcd_next.points = o3d.utility.Vector3dVector(xyz1[:, 0:3])

        pcd_all += pcd_next.voxel_down_sample(voxel_size=voxel_size)

        vis.update_geometry(pcd_all) if (frames > 0) else vis.add_geometry(pcd_all)        
        vis.poll_events()
        vis.update_renderer()

        frames += 1        
except:
    pass

print('Capture stopped')

rx_depth.close()
vis.destroy_window()

o3d.visualization.draw_geometries([pcd_all])
