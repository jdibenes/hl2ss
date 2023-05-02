
import fractions
import numpy as np
import time
import cv2
import av
import hl2ss
import hl2ss_io
import hl2ss_3dcv


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


class microphone_resampler:
    def open(self, target_format=None, target_layout=None, target_rate=None):
        self._resampler = av.AudioResampler(format=target_format, layout=target_layout, rate=target_rate)

    def resample(self, data, format, layout):
        in_frame = av.AudioFrame.from_ndarray(data, format=format, layout=layout)
        out_frames = self._resampler.resample(in_frame)
        return [frame.to_ndarray() for frame in out_frames]


#------------------------------------------------------------------------------
# SI
#------------------------------------------------------------------------------

class _SI_Hand:
    def __init__(self, poses, orientations, positions, radii, accuracies):
        self.poses = poses
        self.orientations = orientations
        self.positions = positions
        self.radii = radii
        self.accuracies = accuracies


def si_unpack_hand(hand):
    poses = [hand.get_joint_pose(joint) for joint in range(0, hl2ss.SI_HandJointKind.TOTAL)]
    orientations = np.array([pose.orientation for pose in poses])
    positions = np.array([pose.position for pose in poses])
    radii = np.array([pose.radius for pose in poses])
    accuracies = np.array([pose.accuracy for pose in poses])
    return _SI_Hand(poses, orientations, positions, radii, accuracies)


def si_head_pose_rotation_matrix(up, forward):
    y = up
    z = -forward
    x = np.cross(y, z)
    return np.hstack((x, y, z)).reshape((3, 3)).transpose()


def si_ray_to_vector(origin, direction):
    return np.vstack((origin, direction)).reshape((-1, 6))


def si_ray_transform(ray, transform4x4):
    ray_t = ray.copy()
    ray_t[:, 0:3] = hl2ss_3dcv.transform(ray[:, 0:3], transform4x4)
    ray_t[:, 3:6] = hl2ss_3dcv.orient(ray[:, 3:6], transform4x4)
    return ray_t


def si_ray_to_point(ray, d):
    return (ray[:, 0:3] + d * ray[:, 3:6]).reshape((-1, 3))


#------------------------------------------------------------------------------
# Draw
#------------------------------------------------------------------------------

def draw_points(image, points, radius, color, thickness):
    for x, y in points:
        if (x >= 0 and y >= 0 and x < image.shape[1] and y < image.shape[0]):
            cv2.circle(image, (x, y), radius, color, thickness)
    return image


#------------------------------------------------------------------------------
# Unpack to MP4
#------------------------------------------------------------------------------

def get_av_codec_name(reader):
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return hl2ss.get_video_codec_name(reader.header.profile)
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return hl2ss.get_video_codec_name(reader.header.profile)
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return hl2ss.get_video_codec_name(reader.header.profile)
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return hl2ss.get_video_codec_name(reader.header.profile)
    if (reader.header.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return hl2ss.get_video_codec_name(reader.header.profile)
    if (reader.header.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return hl2ss.get_video_codec_name(reader.header.profile)
    if (reader.header.port == hl2ss.StreamPort.MICROPHONE):
        return hl2ss.get_audio_codec_name(reader.header.profile)
    

def get_av_framerate(reader):
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (reader.header.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (reader.header.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return hl2ss.Parameters_RM_DEPTH_AHAT.FPS
    if (reader.header.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS
    if (reader.header.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return reader.header.framerate
    if (reader.header.port == hl2ss.StreamPort.MICROPHONE):
        return hl2ss.Parameters_MICROPHONE.SAMPLE_RATE


def unpack_to_mp4(input_filenames, output_filename):
    readers = [hl2ss_io.create_rd(False, input_filename, hl2ss.ChunkSize.SINGLE_TRANSFER, None) for input_filename in input_filenames]
    [reader.open() for reader in readers]

    container = av.open(output_filename, mode='w')
    streams = [container.add_stream(get_av_codec_name(reader), rate=get_av_framerate(reader)) for reader in readers]
    codecs = [av.CodecContext.create(get_av_codec_name(reader), "r") for reader in readers]

    base = hl2ss._RANGEOF.U64_MAX

    for reader in readers:
        data = reader.read()
        if ((data is not None) and (data.timestamp < base)):
            base = data.timestamp

    [reader.close() for reader in readers]
    [reader.open() for reader in readers]

    time_base = fractions.Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)

    for i in range(0, len(input_filenames)):
        while (True):
            data = readers[i].read()
            if (data is None):
                break
            for packet in codecs[i].parse(data.payload):
                packet.stream = streams[i]
                packet.pts = data.timestamp - base
                packet.dts = packet.pts
                packet.time_base = time_base
                container.mux(packet)

    container.close()
    [reader.close() for reader in readers]


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

