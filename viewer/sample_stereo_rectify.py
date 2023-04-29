
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_utilities
import hl2ss_3dcv
import numpy as np

# Settings --------------------------------------------------------------------

host = '192.168.1.7'
port_left  = hl2ss.StreamPort.RM_VLC_LEFTFRONT
port_right = hl2ss.StreamPort.RM_VLC_RIGHTFRONT
calibration_path = '../calibration'

vlc_profile = hl2ss.VideoProfile.H264_BASE
vlc_bitrate = 2*1024*1024

shape = hl2ss.Parameters_RM_VLC.SHAPE

line_start = 10
line_offset = 20
line_color = (0, 255, 0)
line_thickness = 1

#------------------------------------------------------------------------------

# Get camera calibrations

calibration_lf = hl2ss_3dcv.get_calibration_rm(host, port_left, calibration_path)
calibration_rf = hl2ss_3dcv.get_calibration_rm(host, port_right, calibration_path)

rotation_lf = hl2ss_3dcv.rm_vlc_get_rotation(port_left)
rotation_rf = hl2ss_3dcv.rm_vlc_get_rotation(port_right)

K1, Rt1 = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_lf.intrinsics, calibration_lf.extrinsics, rotation_lf)
K2, Rt2 = hl2ss_3dcv.rm_vlc_rotate_calibration(calibration_rf.intrinsics, calibration_rf.extrinsics, rotation_rf)

# Get calibration and rectify

stereo_calibration = hl2ss_3dcv.rm_vlc_stereo_calibrate(K1, K2, Rt1, Rt2)
stereo_rectification = hl2ss_3dcv.rm_vlc_stereo_rectify(K1, K2, stereo_calibration.R, stereo_calibration.t, shape)

# Show rectified images
# To keep this example simple, the images are captured one after the other but this will not work properly for dynamic scenes
# Use the multiprocessing producer to obtain image pairs that are closest in time

client_lf = hl2ss.rx_decoded_rm_vlc(host, port_left,  hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, vlc_profile, vlc_bitrate)
client_rf = hl2ss.rx_decoded_rm_vlc(host, port_right, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, vlc_profile, vlc_bitrate)

client_lf.open()
data_lf = client_lf.get_next_packet()
client_lf.close()

client_rf.open()
data_rf = client_rf.get_next_packet()
client_rf.close()

image_l = hl2ss_3dcv.rm_vlc_to_rgb(hl2ss_3dcv.rm_vlc_rotate_image(cv2.remap(data_lf.payload, calibration_lf.undistort_map[:,:,0], calibration_lf.undistort_map[:,:,1], cv2.INTER_LINEAR), rotation_lf))
image_r = hl2ss_3dcv.rm_vlc_to_rgb(hl2ss_3dcv.rm_vlc_rotate_image(cv2.remap(data_rf.payload, calibration_rf.undistort_map[:,:,0], calibration_rf.undistort_map[:,:,1], cv2.INTER_LINEAR), rotation_rf))

r1 = cv2.remap(image_l, stereo_rectification.map1[:, :, 0], stereo_rectification.map1[:, :, 1], cv2.INTER_LINEAR)
r2 = cv2.remap(image_r, stereo_rectification.map2[:, :, 0], stereo_rectification.map2[:, :, 1], cv2.INTER_LINEAR)

image = np.hstack((r1, r2))

for y in range(line_start, shape[1], line_offset):
    cv2.line(image, (0, y), ((shape[0] * 2) - 1, y), line_color, line_thickness)

cv2.imshow('rect', image)
cv2.waitKey(0)
