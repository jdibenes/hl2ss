# Get PV extrinsics w.r.t. rig from the SE(3) average of multiple stationary samples

from scipy.spatial.transform import Rotation as R

import numpy as np
import hl2ss

host = '192.168.1.15'
samples = 120

calibration_vlc = hl2ss.download_calibration_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT)

client_pv = hl2ss.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, 640, 360, 30, hl2ss.VideoProfile.H264_BASE, 2*1024*1024)
client_vlc = hl2ss.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_1, hl2ss.VideoProfile.H264_BASE, 2*1024*1024)

list_pv = []
list_vlc = []

client_pv.open()
client_vlc.open()

for _ in range(0, samples):
    list_pv.append(client_pv.get_next_packet().pose)
    list_vlc.append(client_vlc.get_next_packet().pose)

client_pv.close()
client_vlc.close()

# camera_to_world
# LEFTFRONT extrinsics = Identity
# inv(extrinsics_pv) @ rm_pose
# pv_pose
# =>
# inv(extrinsics_pv) @ rm_pose = pv_pose
# inv(extrinsics_pv) = pv_pose @ inv(rm_pose)
# extrinsics_pv = rm_pose @ inv(pv_pose)

list_extrinsics = [vlc_pose @ np.linalg.inv(pv_pose) for vlc_pose, pv_pose in zip(list_vlc, list_pv)]
translation = np.mean(np.array([list_extrinsics[i][3, 0:3] for i in range(0, samples)], dtype=np.float32), axis=0)
rotation = R.from_matrix(np.vstack(tuple([list_extrinsics[i][:3,:3].reshape((1, 3, 3)) for i in range(0, samples)]))).mean().as_matrix()

extrinsics = np.zeros((4, 4), dtype=np.float32)
extrinsics[:3, :3] = rotation
extrinsics[ 3, :3] = translation
extrinsics[ 3,  3] = 1

extrinsics.tofile('../calibration/pv/rm_extrinsics.bin')
