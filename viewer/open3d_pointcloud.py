#------------------------------------------------------------------------------
# This script captures a single depth image from the HoloLens and converts it
# to a pointcloud. 3D points are in meters.
#------------------------------------------------------------------------------

import open3d as o3d
import numpy as np
import hl2ss
import av

# Settings --------------------------------------------------------------------

host = '192.168.1.15'
width = 640
height = 360
framerate = 15
profile = hl2ss.VideoProfile.H264_MAIN
bitrate = 5*1024*1024
max_range = 3.5

#------------------------------------------------------------------------------

calibration_depth = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
calibration_pv = hl2ss.download_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width, height, framerate, profile, bitrate)

pixels = hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS

x = calibration_depth.uv2xy[:, :, 0].reshape((pixels, 1))
y = calibration_depth.uv2xy[:, :, 1].reshape((pixels, 1))
z = np.ones((pixels, 1))
n = np.sqrt(x**2 + y**2 + 1)
v = np.hstack((x, y, z)) / n

rx_depth = hl2ss.rx_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1)

rx_depth.open()
data = rx_depth.get_next_packet()
images = hl2ss.unpack_rm_depth(data.payload)
rx_depth.close()

d = images.depth.reshape((pixels, 1)) / calibration_depth.scale
xyz = v * d
xyz = xyz[(d.reshape((pixels,)) > 0) & (d.reshape((pixels,)) < max_range)]
xyz1 = np.hstack((xyz, np.ones((xyz.shape[0], 1)))) @ np.linalg.inv(calibration_depth.extrinsics) @ data.pose

rx_pv = hl2ss.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate)
decoder = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')

rx_pv.open()
for _ in range(0, 2):
    data = rx_pv.get_next_packet()
    for packet in decoder.parse(data.payload):
        for frame in decoder.decode(packet):
            image = frame.to_ndarray(format='rgb24')
rx_pv.close()

xyz1_pv = xyz1 @ np.linalg.inv(data.pose) @ calibration_pv.projection
xyz_pv = xyz1_pv[:, 0:3] / xyz1_pv[:, 2].reshape((xyz.shape[0], 1))
uv_pv = xyz_pv[:, 0:2].astype(int)
select = (uv_pv[:, 0] >= 0) & (uv_pv[:, 0] < width) & (uv_pv[:, 1] >= 0) & (uv_pv[:, 1] < height)
uv_pv = uv_pv[select,:]
xyz = xyz[select, :]

colors = [image[uv[1], uv[0]] for uv in uv_pv]
colors = np.stack(colors, axis=0).astype(float) / 255.0

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd])
