# Invert HL2 image to camera unit plane map

from scipy.spatial import KDTree
from shapely.geometry import Polygon
from shapely.geometry import Point

import numpy as np
import cv2
import hl2ss
import hl2ss_utilities

host = '192.168.1.15'
port = hl2ss.StreamPort.RM_VLC_RIGHTRIGHT

if (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
    calibration = hl2ss.download_calibration_rm_vlc(host, port)
    name = 'rm_vlc_leftfront'
    client = hl2ss_utilities.rx_decoded_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, hl2ss.VideoProfile.H264_MAIN, 1*1024*1024)
elif (port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
    calibration = hl2ss.download_calibration_rm_vlc(host, port)
    name = 'rm_vlc_leftleft'
    client = hl2ss_utilities.rx_decoded_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, hl2ss.VideoProfile.H264_MAIN, 1*1024*1024)
elif (port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
    calibration = hl2ss.download_calibration_rm_vlc(host, port)
    name = 'rm_vlc_rightfront'
    client = hl2ss_utilities.rx_decoded_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, hl2ss.VideoProfile.H264_MAIN, 1*1024*1024)
elif (port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
    calibration = hl2ss.download_calibration_rm_vlc(host, port)
    name = 'rm_vlc_rightright'
    client = hl2ss_utilities.rx_decoded_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, hl2ss.VideoProfile.H264_MAIN, 1*1024*1024)
elif (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
    calibration = hl2ss.download_calibration_rm_depth_longthrow(host, port)
    name = 'rm_depth_longthrow'
    client = hl2ss_utilities.rx_decoded_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_0)
else:
    quit()

src_map = calibration.uv2xy

width = src_map.shape[1]
height = src_map.shape[0]

dst_map = -np.ones((height, width, 2), dtype=np.float32)

src_x = src_map[:, :, 0].reshape((-1, 1))
src_y = src_map[:, :, 1].reshape((-1, 1))

min_x = np.min(src_x)
max_x = np.max(src_x)
min_y = np.min(src_y)
max_y = np.max(src_y)

span_x = max_x - min_x
span_y = max_y - min_y
span_u = width - 1
span_v = height - 1

inverse_K = np.array([[span_x / span_u, 0, 0], [0, span_y / span_v, 0], [min_x, min_y, 1]], dtype=np.float32)
K = np.array([[span_u / span_x, 0, 0, 0], [0, span_v / span_y, 0, 0], [-min_x * span_u / span_x, -min_y * span_v / span_y, 1, 0], [0, 0, 0, 1]], dtype=np.float32)

kd = KDTree(np.hstack((src_x, src_y)))

dst_u, dst_v = np.meshgrid(np.arange(0, width,  1, dtype=np.float32), np.arange(0, height, 1, dtype=np.float32))
dst_xy1 = np.hstack((dst_u.reshape((-1, 1)), dst_v.reshape((-1, 1)), np.ones((width * height, 1), dtype=np.float32))) @ inverse_K

dst_xy = dst_xy1[:, 0:2]
src_d, src_i = kd.query(dst_xy)

def test_quadrant(src_map, dst_xy, src_u, src_v, du, dv):
    uv_ul = [src_u + du, src_v + dv]

    if ((uv_ul[0] < 0) or (uv_ul[1] < 0) or (uv_ul[0] >= (src_map.shape[1] - 1)) or (uv_ul[1] >= (src_map.shape[0] - 1))):
        return (-1,)

    uv_ur = [uv_ul[0] + 1, uv_ul[1] + 0]
    uv_bl = [uv_ul[0] + 0, uv_ul[1] + 1]
    uv_br = [uv_ul[0] + 1, uv_ul[1] + 1]

    xy_ul = src_map[uv_ul[1], uv_ul[0], :]
    xy_ur = src_map[uv_ur[1], uv_ur[0], :]
    xy_bl = src_map[uv_bl[1], uv_bl[0], :]
    xy_br = src_map[uv_br[1], uv_br[0], :]

    return (0 if Polygon([xy_ul, xy_ur, xy_br, xy_bl]).contains(dst_xy) else 1, xy_ul, xy_ur, xy_bl, xy_br, np.array(uv_ul, dtype=np.float32), np.array(uv_ur, dtype=np.float32), np.array(uv_bl, dtype=np.float32), np.array(uv_br, dtype=np.float32))

def find_quadrant(src_map, dst_point, src_u, src_v):
    r0 = test_quadrant(src_map, dst_point, src_u, src_v, -1, -1)
    if (r0[0] == 0):
        return r0
    r1 = test_quadrant(src_map, dst_point, src_u, src_v, -1,  0)
    if (r1[0] == 0):
        return r1
    r2 = test_quadrant(src_map, dst_point, src_u, src_v,  0, -1)
    if (r2[0] == 0):
        return r2
    r3 = test_quadrant(src_map, dst_point, src_u, src_v,  0,  0)
    if (r3[0] == 0):
        return r3
    if (r0[0] == 1 and r1[0] == 1 and r2[0] == 1 and r3[0] == 1):
        print('Point not in any quadrant')
    return (-1,)

for index in range(0, dst_xy.shape[0]):
    point = dst_xy[index, :]

    q = find_quadrant(src_map, Point(point), src_i[index] % width, src_i[index] // width)
    if (q[0] != 0):
        continue

    dp0 = np.linalg.norm(point - q[1])
    dp1 = np.linalg.norm(point - q[2])
    dp2 = np.linalg.norm(point - q[3])
    dp3 = np.linalg.norm(point - q[4])

    d0 = dp1*dp2*dp3
    d1 = dp0*dp2*dp3
    d2 = dp0*dp1*dp3
    d3 = dp0*dp1*dp2

    p0 = q[5]
    p1 = q[6]
    p2 = q[7]
    p3 = q[8]

    dst_map[index // width, index % width, :] = (d0*p0 + d1*p1 + d2*p2 + d3*p3) / (d0 + d1 + d2 + d3)

undistorted_map = np.dstack((dst_xy[:, 0].reshape((height, width, 1)), dst_xy[:, 1].reshape((height, width, 1))))

undistorted_map.tofile(name + '_uv2xy.bin')
dst_map.tofile(name + '_map.bin')
K.tofile(name + '_intrinsics.bin')

client.open()
data = client.get_next_packet()
client.close()

if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
    image = data.payload.depth * 8
else:
    image = data.payload

new_image = cv2.remap(image, dst_map[:,:,0], dst_map[:, :, 1], cv2.INTER_NEAREST)

cv2.imshow('image', image)
cv2.imshow('new image', new_image)
cv2.waitKey(0)
