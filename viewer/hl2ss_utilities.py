
import numpy as np
import os
import cv2
import time
import av
import hl2ss
import hl2ss_mp


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

class SI_Hand:
    def __init__(self, poses, orientations, positions, radii, accuracies):
        self.poses = poses
        self.orientations = orientations
        self.positions = positions
        self.radii = radii
        self.accuracies = accuracies


def si_unpack_hand(hand):
    poses = [hand.get_joint_pose(joint) for joint in range(0, hl2ss.HandJointKind.TOTAL)]
    orientations = np.array([pose.orientation for pose in poses])
    positions = np.array([pose.position for pose in poses])
    radii = np.array([pose.radius for pose in poses])
    accuracies = np.array([pose.accuracy for pose in poses])
    return SI_Hand(poses, orientations, positions, radii, accuracies)


def si_head_pose_rotation_matrix(head_pose):
    y = head_pose.up
    z = -head_pose.forward
    x = np.cross(y, z)
    return np.hstack((x, y, z)).reshape((3, 3)).transpose()


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
        
    def initialize_rm_depth(self, buffer_size, host, port, chunk, mode, png_filter):
        self._add(hl2ss_mp.producer(hl2ss.rx_rm_depth_longthrow(host, port, chunk, mode, png_filter), buffer_size), port)

    def initialize_rm_imu(self, buffer_size, host, port, chunk, mode):
        self._add(hl2ss_mp.producer(hl2ss.rx_rm_imu(host, port, chunk, mode), buffer_size), port)

    def initialize_pv(self, buffer_size, host, port, chunk, mode, width, height, framerate, profile, bitrate):        
        self._add(hl2ss_mp.producer(hl2ss.rx_pv(host, port, chunk, mode, width, height, framerate, profile, bitrate), buffer_size), port)

    def initialize_microphone(self, buffer_size, host, port, chunk, profile):
        self._add(hl2ss_mp.producer(hl2ss.rx_microphone(host, port, chunk, profile), buffer_size), port)

    def initialize_si(self, buffer_size, host, port, chunk):
        self._add(hl2ss_mp.producer(hl2ss.rx_si(host, port, chunk), buffer_size), port)

    def initialize_decoded_rm_vlc(self, buffer_size, host, port, chunk, mode, profile, bitrate):
        self._add(hl2ss_mp.producer(hl2ss.rx_decoded_rm_vlc(host, port, chunk, mode, profile, bitrate), buffer_size), port)

    def initialize_decoded_rm_depth_longthrow(self, buffer_size, host, port, chunk, mode, png_filter):
        self._add(hl2ss_mp.producer(hl2ss.rx_decoded_rm_depth_longthrow(host, port, chunk, mode, png_filter), buffer_size), port)

    def initialize_decoded_pv(self, buffer_size, host, port, chunk, mode, width, height, framerate, profile, bitrate, format):
        self._add(hl2ss_mp.producer(hl2ss.rx_decoded_pv(host, port, chunk, mode, width, height, framerate, profile, bitrate, format), buffer_size), port)

    def initialize_decoded_microphone(self, buffer_size, host, port, chunk, profile):
        self._add(hl2ss_mp.producer(hl2ss.rx_decoded_microphone(host, port, chunk, profile), buffer_size), port)

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

