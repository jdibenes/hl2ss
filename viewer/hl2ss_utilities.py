
import numpy as np
import os
import cv2
import time
import av
import hl2ss
import hl2ss_mp


#------------------------------------------------------------------------------
# Version
#------------------------------------------------------------------------------

class _RANGEOF:
    U8_MAX = 255
    U16_MAX = 65535


def get_server_version(host):
    settings = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    return settings.get_version()


#------------------------------------------------------------------------------
# Decoders
#------------------------------------------------------------------------------

class decoder_vlc:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = av.CodecContext.create(hl2ss.get_video_codec_name(self.profile), 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return frame.to_ndarray(format='bgr24')[:, :, 0]
        return None


class decoder_pv:
    def __init__(self, profile, format):
        self.profile = profile
        self.format = format

    def create(self):
        self._codec = av.CodecContext.create(hl2ss.get_video_codec_name(self.profile), 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return frame.to_ndarray(format=self.format)
        return None


class decoder_microphone:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = av.CodecContext.create(hl2ss.get_audio_codec_name(self.profile), 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return frame.to_ndarray()
        return None


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_rm_vlc:
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        self._client = hl2ss.rx_rm_vlc(host, port, chunk, mode, profile, bitrate)
        self._codec = decoder_vlc(profile)

    def open(self):
        self._codec.create()
        self._client.open()
        self.get_next_packet()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        data.payload = self._codec.decode(data.payload)
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
        self._codec = decoder_pv(profile, format)

    def open(self):        
        self._codec.create()
        self._client.open()
        self.get_next_packet()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        self._client.close()


class rx_decoded_microphone:
    def __init__(self, host, port, chunk, profile):
        self._client = hl2ss.rx_microphone(host, port, chunk, profile)
        self._codec = decoder_microphone(profile)
        
    def open(self):
        self._codec.create()
        self._client.open()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        data.payload = self._codec.decode(data.payload)
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
    def __init__(self, map, uv2xy, intrinsics, extrinsics):
        self.map        = map
        self.uv2xy      = uv2xy
        self.intrinsics = intrinsics
        self.extrinsics = extrinsics


def _rm_load_pinhole_model(path, width, height):
    map = np.fromfile(os.path.join(path, 'map.bin'), dtype=np.float32).reshape((height, width, 2))
    uv2xy = np.fromfile(os.path.join(path, 'uv2xy.bin'), dtype=np.float32).reshape((height, width, 2))
    intrinsics = np.fromfile(os.path.join(path, 'intrinsics.bin'), dtype=np.float32).reshape((4, 4))
    extrinsics = np.fromfile(os.path.join(path, 'extrinsics.bin'), dtype=np.float32).reshape((4, 4))
    return RM_Pinhole_Model(map, uv2xy, intrinsics, extrinsics)


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
    return ab.astype(np.float32) / _RANGEOF.U16_MAX


def rm_depth_ab_to_uint8(ab):
    return (rm_depth_ab_to_float(ab) * _RANGEOF.U8_MAX).astype(np.uint8)


def rm_depth_get_normalizer(uv2xy):
    return np.sqrt(uv2xy[:, :, 0]**2 + uv2xy[:, :, 1]**2 + 1) * hl2ss.Parameters_RM_DEPTH_LONGTHROW.SCALE


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

def pv_optimize_for_cv(host, focus, exposure, color_preset):
    settings = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)

    settings.set_video_temporal_denoising(hl2ss.VideoTemporalDenoisingMode.Off)
    settings.set_focus(hl2ss.FocusMode.Manual, hl2ss.AutoFocusRange.Normal, hl2ss.ManualFocusDistance.Infinity, focus, hl2ss.DriverFallback.Disable)
    settings.set_exposure(hl2ss.ExposureMode.Manual, exposure)
    settings.set_white_balance_preset(color_preset)


def pv_load_rm_extrinsics(path):
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
# Producer
#------------------------------------------------------------------------------

class producer:
    def __init__(self):
        self._producer = dict()

    def _add(self, producer, port):
        self._producer[port] = producer

    def initialize_rm_vlc(self, buffer_size, host, port, chunk, mode, profile, bitrate):
        self._add(hl2ss_mp.producer(hl2ss.rx_rm_vlc(host, port, chunk, mode, profile, bitrate), buffer_size), port)
        
    def initialize_rm_depth(self, buffer_size, host, port, chunk, mode):
        self._add(hl2ss_mp.producer(hl2ss.rx_rm_depth(host, port, chunk, mode), buffer_size), port)

    def initialize_rm_imu(self, buffer_size, host, port, chunk, mode):
        self._add(hl2ss_mp.producer(hl2ss.rx_rm_imu(host, port, chunk, mode), buffer_size), port)

    def initialize_pv(self, buffer_size, host, port, chunk, mode, width, height, framerate, profile, bitrate):        
        self._add(hl2ss_mp.producer(hl2ss.rx_pv(host, port, chunk, mode, width, height, framerate, profile, bitrate), buffer_size), port)

    def initialize_microphone(self, buffer_size, host, port, chunk, profile):
        self._add(hl2ss_mp.producer(hl2ss.rx_microphone(host, port, chunk, profile), buffer_size), port)

    def initialize_si(self, buffer_size, host, port, chunk):
        self._add(hl2ss_mp.producer(hl2ss.rx_si(host, port, chunk), buffer_size), port)

    def initialize_decoded_rm_vlc(self, buffer_size, host, port, chunk, mode, profile, bitrate):
        self._add(hl2ss_mp.producer(rx_decoded_rm_vlc(host, port, chunk, mode, profile, bitrate), buffer_size), port)

    def initialize_decoded_rm_depth(self, buffer_size, host, port, chunk, mode):
        self._add(hl2ss_mp.producer(rx_decoded_rm_depth(host, port, chunk, mode), buffer_size), port)

    def initialize_decoded_pv(self, buffer_size, host, port, chunk, mode, width, height, framerate, profile, bitrate, format):
        self._add(hl2ss_mp.producer(rx_decoded_pv(host, port, chunk, mode, width, height, framerate, profile, bitrate, format), buffer_size), port)

    def initialize_decoded_microphone(self, buffer_size, host, port, chunk, profile):
        self._add(hl2ss_mp.producer(rx_decoded_microphone(host, port, chunk, profile), buffer_size), port)

    def start(self):
        for producer in self._producer.values():
            producer.start()

    def stop(self):
        for producer in self._producer.values():
            producer.stop()

    def create_sink(self, port, sink_din, sink_dout, sink_semaphore):
        return self._producer[port].create_sink(sink_din, sink_dout, sink_semaphore)


#------------------------------------------------------------------------------
# Consumer
#------------------------------------------------------------------------------

class consumer:
    def __init__(self):
        self._sink = dict()
        self._sink_semaphore = dict()

    def create_sink(self, producer, port, manager, semaphore):
        sink_semaphore = None if (semaphore is None) else manager.Semaphore(hl2ss_mp.interconnect.IPC_SEMAPHORE_VALUE) if (semaphore is ...) else self._sink_semaphore[semaphore]
        sink = producer.create_sink(port, manager.Queue(), manager.Queue(), sink_semaphore)
        self._sink[port] = sink
        self._sink_semaphore[port] = sink_semaphore
        return sink





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

