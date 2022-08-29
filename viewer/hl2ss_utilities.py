
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
    return np.hstack((array, np.ones((array.shape[0], 1), dtype=array.dtype)))


def to_inhomogeneous(array):
    end = array.shape[1] - 1
    z = array[:, end].reshape((-1, 1))
    return (array[:, 0:end] / z, z)


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


def rm_load_pinhole_model(path, width, height):
    intrinsics = np.fromfile(os.path.join(path, 'intrinsics.bin'), dtype=np.float32).reshape((4, 4))
    map = np.fromfile(os.path.join(path, 'map.bin'), dtype=np.float32).reshape((height, width, 2))
    uv2xy = np.fromfile(os.path.join(path, 'uv2xy.bin'), dtype=np.float32).reshape((height, width, 2))
    return RM_Pinhole_Model(map, intrinsics, uv2xy)


def rm_camera_to_world(extrinsics, pose):
    return np.linalg.inv(extrinsics) @ pose


def rm_world_to_camera(extrinsics, pose):
    return np.linalg.inv(pose) @ extrinsics


#------------------------------------------------------------------------------
# RM VLC
#------------------------------------------------------------------------------

def rm_vlc_load_pinhole_model(path):
    return rm_load_pinhole_model(path, hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT)


def rm_vlc_undistort(image, map):
    return cv2.remap(image, map[:, :, 0], map[:, :, 1], cv2.INTER_LINEAR)


#------------------------------------------------------------------------------
# RM Depth
#------------------------------------------------------------------------------

def rm_depth_longthrow_load_pinhole_model(path):
    return rm_load_pinhole_model(path, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)


def rm_depth_get_normalizer(uv2xy, scale):
    uv2x = uv2xy[:, :, 0]
    uv2y = uv2xy[:, :, 1]
    return np.sqrt(uv2x**2 + uv2y**2 + 1) * scale


def rm_depth_to_points(depth, uv2xy):
    x = uv2xy[:, :, 0] * depth
    y = uv2xy[:, :, 1] * depth
    z = depth
    return np.hstack((x.reshape((-1, 1)), y.reshape((-1, 1)), z.reshape((-1, 1))))


def rm_depth_undistort(depth, map):
    return cv2.remap(depth, map[:, :, 0], map[:, :, 1], cv2.INTER_NEAREST)


def rm_depth_generate_rgbd(rgb, depth, rgb_projection, depth_uv2xy, depth_camera_to_world):
    uv, _ = project_to_image(to_homogeneous(rm_depth_to_points(depth, depth_uv2xy)) @ depth_camera_to_world, rgb_projection)
    u = uv[:, 0].reshape(depth.shape)
    v = uv[:, 1].reshape(depth.shape)
    aligned_rgb = cv2.remap(rgb, u, v, cv2.INTER_LINEAR)
    depth[(u < 0) | (u > (rgb.shape[1] - 1)) | (v < 0) | (v > (rgb.shape[0] - 1))] = 0    
    return (aligned_rgb, depth)


#------------------------------------------------------------------------------
# PV
#------------------------------------------------------------------------------

def pv_camera_to_word(pose):
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

