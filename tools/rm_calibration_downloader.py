# RM calibration downloader

import hl2ss

host = '192.168.1.15'

calibration_depth  = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
calibration_vlc_lf = hl2ss.download_calibration_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT)
calibration_vlc_ll = hl2ss.download_calibration_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTLEFT)
calibration_vlc_rf = hl2ss.download_calibration_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT)
calibration_vlc_rr = hl2ss.download_calibration_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT)

calibration_depth.extrinsics.tofile('rm_depth_longthrow_extrinsics.bin')
calibration_depth.uv2xy.tofile('rm_depth_longthrow_uv2xy.bin')
calibration_depth.scale.tofile('rm_depth_longthrow_scale.bin')

calibration_vlc_lf.extrinsics.tofile('rm_vlc_leftfront_extrinsics.bin')
calibration_vlc_lf.uv2xy.tofile('rm_vlc_leftfront_uv2xy.bin')

calibration_vlc_ll.extrinsics.tofile('rm_vlc_leftleft_extrinsics.bin')
calibration_vlc_ll.uv2xy.tofile('rm_vlc_leftleft_uv2xy.bin')

calibration_vlc_rf.extrinsics.tofile('rm_vlc_rightfront_extrinsics.bin')
calibration_vlc_rf.uv2xy.tofile('rm_vlc_rightfront_uv2xy.bin')

calibration_vlc_rr.extrinsics.tofile('rm_vlc_rightright_extrinsics.bin')
calibration_vlc_rr.uv2xy.tofile('rm_vlc_rightright_uv2xy.bin')
