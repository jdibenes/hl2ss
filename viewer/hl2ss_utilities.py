
import numpy as np
import os
import cv2
import time
import av
import hl2ss


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_rm_vlc:
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        self._client = hl2ss.rx_rm_vlc(host, port, chunk, mode, profile, bitrate)
        self._codec_name = hl2ss.get_video_codec_name(profile)

    def open(self):
        self._codec = av.CodecContext.create(self._codec_name, 'r')
        self._client.open()
        self.get_next_packet()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        for packet in self._codec.parse(data.payload):
            for frame in self._codec.decode(packet):
                data.payload = frame.to_ndarray(format='bgr24')[:, :, 0]
        return data

    def close(self):
        self._client.close()


class rx_decoded_rm_depth:
    def __init__(self, host, port, chunk, mode):
        self._client = hl2ss.rx_rm_depth(host, port, chunk, mode)

    def open(self):
        self._client.open()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        data.payload = hl2ss.unpack_rm_depth(data.payload)
        return data

    def close(self):
        self._client.close()


class rx_decoded_pv:
    def __init__(self, host, port, chunk, mode, width, height, framerate, profile, bitrate, format):
        self._client = hl2ss.rx_pv(host, port, chunk, mode, width, height, framerate, profile, bitrate)
        self._codec_name = hl2ss.get_video_codec_name(profile)
        self._format = format

    def open(self):
        self._codec = av.CodecContext.create(self._codec_name, 'r')
        self._client.open()
        self.get_next_packet()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        for packet in self._codec.parse(data.payload):
            for frame in self._codec.decode(packet):
                data.payload = frame.to_ndarray(format=self._format)
        return data

    def close(self):
        self._client.close()


class rx_decoded_microphone:
    def __init__(self, host, port, chunk, profile):
        self._client = hl2ss.rx_microphone(host, port, chunk, profile)
        self._codec_name = hl2ss.get_audio_codec_name(profile)

    def open(self):
        self._codec = av.CodecContext.create(self._codec_name, 'r')
        self._client.open()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        for packet in self._codec.parse(data.payload):
            for frame in self._codec.decode(packet):
                data.payload = frame.to_ndarray()
        return data

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Transforms
#------------------------------------------------------------------------------

def to_homogeneous(array):
    return np.concatenate((array, np.ones(array.shape[0:-1] + (1,), dtype=array.dtype)), axis=-1)


def to_inhomogeneous(array):
    w = array[..., -1, np.newaxis]
    return (array[..., 0:-1] / w, w)


def projection(intrinsics, world_to_camera):
    return world_to_camera @ intrinsics


def project_to_image(hwpoints, projection):
    return to_inhomogeneous((hwpoints @ projection)[:, 0:3])


#------------------------------------------------------------------------------
# RM
#------------------------------------------------------------------------------

class RM_Pinhole_Model:
    def __init__(self, map, intrinsics, uv2xy):
        self.map        = map
        self.intrinsics = intrinsics
        self.uv2xy      = uv2xy


def _rm_load_pinhole_model(path, width, height):
    intrinsics = np.fromfile(os.path.join(path, 'intrinsics.bin'), dtype=np.float32).reshape((4, 4))
    map = np.fromfile(os.path.join(path, 'map.bin'), dtype=np.float32).reshape((height, width, 2))
    uv2xy = np.fromfile(os.path.join(path, 'uv2xy.bin'), dtype=np.float32).reshape((height, width, 2))
    return RM_Pinhole_Model(map, intrinsics, uv2xy)


def rm_camera_to_world(extrinsics, pose):
    return np.linalg.inv(extrinsics) @ pose


def rm_world_to_camera(extrinsics, pose):
    return np.linalg.inv(pose) @ extrinsics


def rm_camera_to_camera(extrinsics_source, extrinsics_destination):
    return np.linalg.inv(extrinsics_source) @ extrinsics_destination


#------------------------------------------------------------------------------
# RM VLC
#------------------------------------------------------------------------------

def rm_vlc_load_pinhole_model(path):
    return _rm_load_pinhole_model(path, hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT)


def rm_vlc_undistort(image, map):
    return cv2.remap(image, map[:, :, 0], map[:, :, 1], cv2.INTER_LINEAR)


#------------------------------------------------------------------------------
# RM Depth
#------------------------------------------------------------------------------

def rm_depth_longthrow_load_pinhole_model(path):
    return _rm_load_pinhole_model(path, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)


def rm_depth_ab_to_float(ab):
    return ab.astype(np.float32) / 65535


def rm_depth_ab_to_uint8(ab):
    return (rm_depth_ab_to_float(ab) * 255).astype(np.uint8)


def rm_depth_get_normalizer(uv2xy, scale):
    return np.sqrt(uv2xy[:, :, 0]**2 + uv2xy[:, :, 1]**2 + 1) * scale


def rm_depth_to_points(depth, uv2xy):
    xyz = uv2xy * depth[:, :, np.newaxis]
    return np.hstack((xyz[:, :, 0].reshape((-1, 1)), xyz[:, :, 1].reshape((-1, 1)), depth.reshape((-1, 1))))


def rm_depth_undistort(depth, map):
    return cv2.remap(depth, map[:, :, 0], map[:, :, 1], cv2.INTER_NEAREST)


def rm_depth_generate_rgbd(ab, depth, depth_map, depth_scale):
    ab = rm_depth_undistort(ab, depth_map)
    depth = rm_depth_undistort(depth, depth_map) / depth_scale
    return (rm_depth_ab_to_uint8(ab), depth)


def rm_depth_generate_rgbd_from_pv(pv, depth, pv_intrinsics, pv_extrinsics, depth_map, depth_scale, depth_uv2xy, depth_extrinsics):
    depth = rm_depth_undistort(depth, depth_map) / depth_scale
    uv, _ = project_to_image(to_homogeneous(rm_depth_to_points(depth, depth_uv2xy)), projection(pv_intrinsics, rm_camera_to_camera(depth_extrinsics, pv_extrinsics)))
    u = uv[:, 0].reshape(depth.shape)
    v = uv[:, 1].reshape(depth.shape)
    depth[(u < 0) | (u > (pv.shape[1] - 1)) | (v < 0) | (v > (pv.shape[0] - 1))] = 0
    pv = cv2.remap(pv, u, v, cv2.INTER_LINEAR)
    return (pv, depth)


def rm_depth_generate_rgbd_from_rm_vlc(vlc, depth, vlc_map, vlc_intrinsics, vlc_extrinsics, depth_map, depth_scale, depth_uv2xy, depth_extrinsics):
    vlc = rm_vlc_undistort(vlc, vlc_map)
    depth = rm_depth_undistort(depth, depth_map) / depth_scale
    uv, _ = project_to_image(to_homogeneous(rm_depth_to_points(depth, depth_uv2xy)), projection(vlc_intrinsics, rm_camera_to_camera(depth_extrinsics, vlc_extrinsics)))
    u = uv[:, 0].reshape(depth.shape)
    v = uv[:, 1].reshape(depth.shape)
    depth[(u < 0) | (u > (vlc.shape[1] - 1)) | (v < 0) | (v > (vlc.shape[0] - 1))] = 0
    vlc = cv2.remap(vlc, u, v, cv2.INTER_LINEAR)
    return (vlc, depth)


#------------------------------------------------------------------------------
# PV
#------------------------------------------------------------------------------

def pv_load_extrinsics(path):
    return np.fromfile(os.path.join(path, 'rm_extrinsics.bin'), dtype=np.float32).reshape((4, 4))


def pv_camera_to_world(pose):
    return pose


def pv_world_to_camera(pose):
    return np.linalg.inv(pose) 


#------------------------------------------------------------------------------
# Microphone
#------------------------------------------------------------------------------

def microphone_planar_to_packed(array):
    data = np.zeros((1, array.size), dtype=array.dtype)
    data[0, 0::2] = array[0, :]
    data[0, 1::2] = array[1, :]
    return data


def microphone_packed_to_planar(array):
    data = np.zeros((2, array.size // 2), dtype=array.dtype)
    data[0, :] = array[0, 0::2]
    data[1, :] = array[0, 1::2]
    return data


#------------------------------------------------------------------------------
# SI
#------------------------------------------------------------------------------

def si_unpack_hand(hand):
    poses = [hand.get_joint_pose(joint) for joint in range(0, hl2ss.HandJointKind.TOTAL)]
    orientations = np.array([pose.orientation for pose in poses])
    positions = np.array([pose.position for pose in poses])
    radii = np.array([pose.radius for pose in poses])
    accuracies = np.array([pose.accuracy for pose in poses])
    return (poses, orientations, positions, radii, accuracies)


#------------------------------------------------------------------------------
# Math
#------------------------------------------------------------------------------

def clamp(v, min, max):
    return min if (v < min) else max if (v > max) else v





#------------------------------------------------------------------------------
# Timing
#------------------------------------------------------------------------------

class continuity_analyzer:
    def __init__(self, period):
        self._period = period
        self._ub = 1.5 * self._period
        self._lb = 0.5 * self._period
        self._last = None

    def push(self, timestamp):
        if (self._last is None):
            status = (0, -1)
        else:
            delta = timestamp - self._last
            status = (1, delta) if (delta > self._ub) else (-1, delta) if (delta < self._lb) else (0, delta)
        self._last = timestamp
        return status


class framerate_counter:
    def reset(self):
        self._count = 0
        self._start = time.perf_counter()

    def increment(self):
        self._count += 1
        return self._count

    def delta(self):
        return time.perf_counter() - self._start

    def get(self):
        return self._count / self.delta()


class stream_report:
    def __init__(self, notify_period, stream_period):
        self._np = notify_period
        self._ca = continuity_analyzer(stream_period)
        self._fc = framerate_counter()
        self._fc.reset()

    def _report_continuity(self, timestamp):
        status, delta = self._ca.push(timestamp)
        if (status != 0):
            print('Discontinuity detected with delta time {delta}'.format(delta=delta))

    def _report_framerate_and_pose(self, timestamp, pose):
        self._fc.increment()
        if (self._fc.delta() >= self._np):
            print('FPS: {fps}'.format(fps=self._fc.get()))
            print('Pose at {timestamp}'.format(timestamp=timestamp))
            print(pose)
            self._fc.reset()

    def push(self, data):
        self._report_continuity(data.timestamp)
        self._report_framerate_and_pose(data.timestamp, data.pose)

