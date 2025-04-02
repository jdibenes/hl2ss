
from pynput import keyboard

import multiprocessing as mp
import types
import io
import fractions
import tarfile
import csv
import numpy as np
import time
import cv2
import av
import queue
import pyaudio
import hl2ss
import hl2ss_mp
import hl2ss_lnm
import hl2ss_io
import hl2ss_3dcv


def depth_colormap(depth, max_depth, colormap=cv2.COLORMAP_JET):
    return cv2.applyColorMap(((depth / max_depth) * 255).astype(np.uint8), colormap)

#------------------------------------------------------------------------------
# Key Listener
#------------------------------------------------------------------------------

class key_listener:
    def __init__(self, key):
        self._key = key

    def _on_press(self, key):
        self._pressed = key == self._key
        return not self._pressed

    def open(self):
        self._pressed = False
        self._listener = keyboard.Listener(on_press=types.MethodType(key_listener._on_press, self))
        self._listener.start()

    def pressed(self):
        return self._pressed

    def close(self):
        self._pressed = True
        self._listener.join()


#------------------------------------------------------------------------------
# Background Writers
#------------------------------------------------------------------------------

class wr_process_rx(mp.Process):
    def __init__(self, filename, rx, user):
        super().__init__()
        self._event_stop = mp.Event()
        self._wr = hl2ss_io.create_wr_from_rx(filename, rx, user)
        self._rx = rx

    def stop(self):
        self._event_stop.set()

    def on_open(self):
        pass

    def on_receive(self, data):
        pass

    def on_close(self):
        pass

    def run(self):
        self.on_open()
        self._wr.open()
        self._rx.open()
        while (not self._event_stop.is_set()):
            data = self._rx.get_next_packet()
            self._wr.write(data)
            self.on_receive(data)
        self._rx.close()
        self._wr.close()
        self.on_close()


class wr_process_producer(mp.Process):
    def __init__(self, filename, producer, port, user):
        super().__init__()
        self._event_stop = mp.Event()
        self._wr = hl2ss_io.create_wr_from_rx(filename, producer.get_receiver(port), user)
        self._sink = hl2ss_mp.consumer().create_sink(producer, port, mp.Manager(), ...)
        self._sync_period = hl2ss_lnm.get_sync_period(self._wr)

    def stop(self):
        self._event_stop.set()
        self._sink.release()
       
    def on_open(self):
        pass

    def on_receive(self, data):
        pass

    def on_fail(self):
        pass

    def on_close(self):
        pass

    def run(self):
        self._frame_stamp = hl2ss_lnm.get_sync_frame_stamp(self._sink.get_attach_response() + 1, self._sync_period)
        self._stopping = False

        self.on_open()
        self._wr.open()

        while ((not self._stopping) or (self._frame_stamp < self._stop_stamp)):
            self._sink.acquire()
            state, _, data = self._sink.get_buffered_frame(self._frame_stamp)

            if (state == 0):
                self._frame_stamp += 1
                self._wr.write(data)
                self.on_receive(data)
            elif (state < 0):
                self.on_fail()
                break

            if ((not self._stopping) and self._event_stop.is_set()):
                self._stopping = True
                self._stop_stamp = self._sink.get_frame_stamp()

        self._wr.close()
        self._sink.detach()
        self.on_close()


#------------------------------------------------------------------------------
# Microphone
#------------------------------------------------------------------------------

class audio_player:
    def __init__(self, subtype, planar, channels, sample_rate):
        self.subtype     = subtype
        self.planar      = planar
        self.channels    = channels
        self.sample_rate = sample_rate

    def open(self):
        self._subtype     = self.subtype
        self._planar      = self.planar
        self._channels    = self.channels
        self._sample_rate = self.sample_rate
        self._fade_in     = 0.0

        self._pcm_queue        = queue.Queue()
        self._pcm_audio_buffer = np.empty((1, 0), dtype=self.subtype)
        self._pcm_ts_buffer    = np.empty((1, 0), dtype=np.int64)
        self._presentation_clk = 0

        self._audio_format = pyaudio.paFloat32 if (self.subtype == np.float32) else pyaudio.paInt16 if (self.subtype == np.int16) else None
        self._p            = pyaudio.PyAudio()
        self._stream       = self._p.open(format=self._audio_format, channels=self.channels, rate=self.sample_rate, output=True, stream_callback=types.MethodType(audio_player._pcm_callback, self))

    def put(self, timestamp, samples):
        self._pcm_queue.put((timestamp, samples))

    def get_timestamp(self):
        return self._presentation_clk

    def _pcm_callback(self, in_data, frame_count, time_info, status):
        samples = self._channels * frame_count

        while (self._pcm_ts_buffer.size < frame_count):
            pcm_data = self._pcm_queue.get()

            if (pcm_data is None):
                return (b'', pyaudio.paAbort)
            
            pcm_timestamp = pcm_data[0]
            pcm_payload   = pcm_data[1]

            pcm_samples    = hl2ss.microphone_planar_to_packed(hl2ss.microphone_packed_to_planar(hl2ss.microphone_planar_to_packed(pcm_payload, self._channels), self._channels), self._channels) if (self._planar) else pcm_payload
            pcm_group_size = pcm_samples.size // self._channels
            pcm_ts         = (pcm_timestamp + (np.arange(0, pcm_group_size, 1, dtype=np.int64) * ((pcm_group_size * hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS) // self._sample_rate))).reshape((1, -1))
            
            self._pcm_audio_buffer = np.hstack((self._pcm_audio_buffer, pcm_samples))
            self._pcm_ts_buffer    = np.hstack((self._pcm_ts_buffer,    pcm_ts))

        gain = self._fade_in
        if (self._fade_in < 1.0):
            self._fade_in += frame_count / (1 * self._sample_rate)

        out_samples = self._pcm_audio_buffer[:, 0:samples] if (gain >= 1.0) else np.zeros((1, samples), dtype=self._subtype)

        self._presentation_clk = self._pcm_ts_buffer[0, 0]
        self._pcm_audio_buffer = self._pcm_audio_buffer[:, samples:]
        self._pcm_ts_buffer    = self._pcm_ts_buffer[:, frame_count:]

        return (out_samples.tobytes(), pyaudio.paContinue)
        
    def close(self):
        self._pcm_queue.put(None)
        self._stream.close()


class microphone_resampler:
    def create(self, target_format=None, target_layout=None, target_rate=None):
        self._resampler = av.AudioResampler(format=target_format, layout=target_layout, rate=target_rate)

    def resample(self, data, profile):
        in_frame = av.AudioFrame.from_ndarray(data, format='s16' if (profile == hl2ss.AudioProfile.RAW) else 'fltp', layout='stereo')
        in_frame.rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
        out_frames = self._resampler.resample(in_frame)
        return [frame.to_ndarray() for frame in out_frames]


#------------------------------------------------------------------------------
# SI
#------------------------------------------------------------------------------

#class _SI_Hand:
#    def __init__(self, poses, orientations, positions, radii, accuracies):
#        self.poses = poses
#        self.orientations = orientations
#        self.positions = positions
#        self.radii = radii
#        self.accuracies = accuracies


#def si_unpack_hand(hand):
#    poses = [hand.get_joint_pose(joint) for joint in range(0, hl2ss.SI_HandJointKind.TOTAL)]
#    orientations = np.array([pose.orientation for pose in poses])
#    positions = np.array([pose.position for pose in poses])
#    radii = np.array([pose.radius for pose in poses])
#    accuracies = np.array([pose.accuracy for pose in poses])
#    return _SI_Hand(poses, orientations, positions, radii, accuracies)




#------------------------------------------------------------------------------
# Draw
#------------------------------------------------------------------------------

def draw_points(image, points, radius, color, thickness):
    for x, y in points:
        if (x >= 0 and y >= 0 and x < image.shape[1] and y < image.shape[0]):
            cv2.circle(image, (x, y), radius, color, thickness)
    return image


#------------------------------------------------------------------------------
# Unpacking
#------------------------------------------------------------------------------

def _create_csv_header_for_timestamp():
    return ['timestamp']


def _create_csv_header_for_pose():
    return [f'pose {i}{j}' for i in range(0, 4) for j in range(0, 4)]


def _create_csv_header_for_rm_imu_sample(index):
    return [f'sensor ticks {index}', f'soc ticks {index}', f'x {index}', f'y {index}', f'z {index}', f'temperature {index}']


def _create_csv_header_for_rm_imu_payload(port):
    header = []
    for i in range(0, hl2ss.rm_imu_get_batch_size(port)):
        header.extend(_create_csv_header_for_rm_imu_sample(i))
    return header


def _create_csv_header_for_pv_payload():
    return ['fx', 'fy', 'cx', 'cy']


def _create_csv_header_for_si_head_pose():
    return ['head pose valid'] + [f'head position {w}' for w in ['x', 'y', 'z']] + [f'head forward {w}' for w in ['x', 'y', 'z']] + [f'head up {w}' for w in ['x', 'y', 'z']]


def _create_csv_header_for_si_eye_ray():
    return ['eye ray valid'] + [f'eye ray origin {w}'for w in ['x', 'y', 'z']] + [f'eye ray direction {w}' for w in ['x', 'y', 'z']]


def _create_csv_header_for_si_hand_joint(hand_name, joint_kind):
    joint_name = hl2ss.si_get_joint_name(joint_kind)
    return [f'{hand_name} {joint_name} position {u}' for u in ['x', 'y', 'z']] + [f'{hand_name} {joint_name} orientation {u}' for u in ['x', 'y', 'z', 'w']] + [f'{hand_name} {joint_name} radius'] + [f'{hand_name} {joint_name} accuracy']


def _create_csv_header_for_si_hand(hand_name):
    header = [f'hand {hand_name} valid']
    for joint_index in range(0, hl2ss.SI_HandJointKind.TOTAL):
        header.extend(_create_csv_header_for_si_hand_joint(hand_name, joint_index))
    return header


def _create_csv_header_for_si_payload():
    return _create_csv_header_for_si_head_pose() + _create_csv_header_for_si_eye_ray() + _create_csv_header_for_si_hand('left') + _create_csv_header_for_si_hand('right')


def _create_csv_header_for_eet_calibration():
    return ['calibration valid']


def _create_csv_header_for_eet_ray(ray_name):
    return [f'{ray_name} valid'] + [f'{ray_name} origin {w}'for w in ['x', 'y', 'z']] + [f'{ray_name} direction {w}' for w in ['x', 'y', 'z']]


def _create_csv_header_for_eet_field(field_name):
    return [f'{field_name} valid', f'{field_name} value']


def _create_csv_header_for_eet_payload():
    return  _create_csv_header_for_eet_calibration() + _create_csv_header_for_eet_ray('combined') + _create_csv_header_for_eet_ray('left') + _create_csv_header_for_eet_ray('right') + _create_csv_header_for_eet_field('left openness') + _create_csv_header_for_eet_field('right openness') + _create_csv_header_for_eet_field('vergence distance')


def _create_csv_header_for_rm_vlc():
    return _create_csv_header_for_timestamp() + _create_csv_header_for_pose()


def _create_csv_header_for_rm_depth():
    return _create_csv_header_for_timestamp() + _create_csv_header_for_pose()


def _create_csv_header_for_rm_imu(port):
    return _create_csv_header_for_timestamp() + _create_csv_header_for_rm_imu_payload(port) + _create_csv_header_for_pose()


def _create_csv_header_for_pv():
    return _create_csv_header_for_timestamp() + _create_csv_header_for_pv_payload() + _create_csv_header_for_pose()


def _create_csv_header_for_microphone():
    return _create_csv_header_for_timestamp()


def _create_csv_header_for_si():
    return _create_csv_header_for_timestamp() + _create_csv_header_for_si_payload()


def _create_csv_header_for_eet():
    return _create_csv_header_for_timestamp() + _create_csv_header_for_eet_payload() + _create_csv_header_for_pose()


def _create_csv_header_for_extended_audio():
    return _create_csv_header_for_timestamp()


def _create_csv_row_for_timestamp(timestamp):
    return [str(timestamp)]


def _create_csv_row_for_pose(pose):
    if (pose is None):
        pose = np.zeros((4, 4), dtype=np.float32)
    return pose.astype(str).flatten().tolist()


def _create_csv_row_for_rm_imu_frame(frame):
    return [str(frame.vinyl_hup_ticks), str(frame.soc_ticks), str(frame.x), str(frame.y), str(frame.z), str(frame.temperature)]


def _create_csv_row_for_rm_imu_payload(payload):
    frames = []
    for i in range(0, payload.get_count()):
        frames.extend(_create_csv_row_for_rm_imu_frame(payload.get_frame(i)))
    return frames


def _create_csv_row_for_pv_payload(payload):
    return payload.focal_length.astype(str).tolist() + payload.principal_point.astype(str).tolist()


def _create_csv_row_for_si_head_pose(valid, pose):
    return valid.astype(str).tolist() + pose.position.astype(str).tolist() + pose.forward.astype(str).tolist() + pose.up.astype(str).tolist()


def _create_csv_row_for_si_eye_ray(valid, ray):
    return valid.astype(str).tolist() + ray.origin.astype(str).tolist() + ray.direction.astype(str).tolist()


def _create_csv_row_for_si_hand_joint(pose):
    return pose.position.astype(str).tolist() + pose.orientation.astype(str).tolist() + pose.radius.astype(str).tolist() + pose.accuracy.astype(str).tolist()


def _create_csv_row_for_si_hand(valid, hand):
    row = valid.astype(str).tolist()
    for joint_index in range(0, hl2ss.SI_HandJointKind.TOTAL):
        row.extend(_create_csv_row_for_si_hand_joint(hand.get_joint_pose(joint_index)))
    return row


def _create_csv_row_for_si_payload(payload):
    return _create_csv_row_for_si_head_pose(payload.head_pose_valid, payload.head_pose) + _create_csv_row_for_si_eye_ray(payload.eye_ray_valid, payload.eye_ray) + _create_csv_row_for_si_hand(payload.hand_left_valid, payload.hand_left) + _create_csv_row_for_si_hand(payload.hand_right_valid, payload.hand_right)


def _create_csv_row_for_eet_calibration(valid):
    return [str(valid)]


def _create_csv_row_for_eet_ray(valid, ray):
    return [str(valid)] + ray.origin.astype(str).tolist() + ray.direction.astype(str).tolist()


def _create_csv_row_for_eet_field(valid, value):
    return [str(valid)] + [value.astype(str).tolist()]


def _create_csv_row_for_eet_payload(payload):
    return _create_csv_row_for_eet_calibration(payload.calibration_valid) + _create_csv_row_for_eet_ray(payload.combined_ray_valid, payload.combined_ray) + _create_csv_row_for_eet_ray(payload.left_ray_valid, payload.left_ray) + _create_csv_row_for_eet_ray(payload.right_ray_valid, payload.right_ray) + _create_csv_row_for_eet_field(payload.left_openness_valid, payload.left_openness) + _create_csv_row_for_eet_field(payload.right_openness_valid, payload.right_openness) + _create_csv_row_for_eet_field(payload.vergence_distance_valid, payload.vergence_distance)


def _create_csv_row_for_rm_vlc(data):
    return _create_csv_row_for_timestamp(data.timestamp) + _create_csv_row_for_pose(data.pose)


def _create_csv_row_for_rm_depth(data):
    return _create_csv_row_for_timestamp(data.timestamp) + _create_csv_row_for_pose(data.pose)


def _create_csv_row_for_rm_imu(data):
    return _create_csv_row_for_timestamp(data.timestamp) + _create_csv_row_for_rm_imu_payload(data.payload) + _create_csv_row_for_pose(data.pose)


def _create_csv_row_for_pv(data):
    return _create_csv_row_for_timestamp(data.timestamp) + _create_csv_row_for_pv_payload(data.payload) + _create_csv_row_for_pose(data.pose)


def _create_csv_row_for_microphone(data):
    return _create_csv_row_for_timestamp(data.timestamp)


def _create_csv_row_for_si(data):
    return _create_csv_row_for_timestamp(data.timestamp) + _create_csv_row_for_si_payload(data.payload)


def _create_csv_row_for_eet(data):
    return _create_csv_row_for_timestamp(data.timestamp) + _create_csv_row_for_eet_payload(data.payload) + _create_csv_row_for_pose(data.pose)


def _create_csv_row_for_extended_audio(data):
    return _create_csv_row_for_timestamp(data.timestamp)


def _create_csv_header(port):
    if (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return _create_csv_header_for_rm_vlc()
    if (port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return _create_csv_header_for_rm_vlc()
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return _create_csv_header_for_rm_vlc()
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return _create_csv_header_for_rm_vlc()
    if (port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return _create_csv_header_for_rm_depth()
    if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return _create_csv_header_for_rm_depth()
    if (port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return _create_csv_header_for_rm_imu(port)
    if (port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return _create_csv_header_for_rm_imu(port)
    if (port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return _create_csv_header_for_rm_imu(port)
    if (port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return _create_csv_header_for_pv()
    if (port == hl2ss.StreamPort.MICROPHONE):
        return _create_csv_header_for_microphone()
    if (port == hl2ss.StreamPort.SPATIAL_INPUT):
        return _create_csv_header_for_si()
    if (port == hl2ss.StreamPort.EXTENDED_EYE_TRACKER):
        return _create_csv_header_for_eet()
    if (port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return _create_csv_header_for_extended_audio()
    if (port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return _create_csv_header_for_pv()


def _create_csv_row(port, data):
    if (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return _create_csv_row_for_rm_vlc(data)
    if (port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return _create_csv_row_for_rm_vlc(data)
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return _create_csv_row_for_rm_vlc(data)
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return _create_csv_row_for_rm_vlc(data)
    if (port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return _create_csv_row_for_rm_depth(data)
    if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return _create_csv_row_for_rm_depth(data)
    if (port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        data.payload = hl2ss.unpack_rm_imu(data.payload)
        return _create_csv_row_for_rm_imu(data)
    if (port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        data.payload = hl2ss.unpack_rm_imu(data.payload)
        return _create_csv_row_for_rm_imu(data)
    if (port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        data.payload = hl2ss.unpack_rm_imu(data.payload)
        return _create_csv_row_for_rm_imu(data)
    if (port == hl2ss.StreamPort.PERSONAL_VIDEO):
        data.payload = hl2ss.unpack_pv(data.payload)
        return _create_csv_row_for_pv(data)
    if (port == hl2ss.StreamPort.MICROPHONE):
        return _create_csv_row_for_microphone(data)
    if (port == hl2ss.StreamPort.SPATIAL_INPUT):
        data.payload = hl2ss.unpack_si(data.payload)
        return _create_csv_row_for_si(data)
    if (port == hl2ss.StreamPort.EXTENDED_EYE_TRACKER):
        data.payload = hl2ss.unpack_eet(data.payload)
        return _create_csv_row_for_eet(data)
    if (port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return _create_csv_row_for_extended_audio(data)
    if (port == hl2ss.StreamPort.EXTENDED_VIDEO):
        data.payload = hl2ss.unpack_pv(data.payload)
        return _create_csv_row_for_pv(data)


def unpack_to_csv(input_filename, output_filename):    
    rd = hl2ss_io.create_rd(input_filename, hl2ss.ChunkSize.SINGLE_TRANSFER, None)
    rd.open()

    port = rd.port

    wr = open(output_filename, 'w', newline='')
    csv_wr = csv.writer(wr)
    csv_wr.writerow(_create_csv_header(port))

    while (True):
        data = rd.get_next_packet()
        if (data is None):
            break
        csv_wr.writerow(_create_csv_row(port, data))

    wr.close()
    rd.close()


def get_av_codec_name(port, profile):
    if (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return hl2ss.get_video_codec_name(profile)
    if (port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return hl2ss.get_video_codec_name(profile)
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return hl2ss.get_video_codec_name(profile)
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return hl2ss.get_video_codec_name(profile)
    if (port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return hl2ss.get_video_codec_name(profile)
    if (port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return hl2ss.get_video_codec_name(profile)
    if (port == hl2ss.StreamPort.MICROPHONE):
        return hl2ss.get_audio_codec_name(profile)
    if (port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return hl2ss.get_audio_codec_name(profile)
    if (port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return hl2ss.get_video_codec_name(profile)


def get_av_framerate(port):
    if (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return hl2ss.Parameters_RM_DEPTH_AHAT.FPS
    if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS
    if (port == hl2ss.StreamPort.MICROPHONE):
        return hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
    if (port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return hl2ss.Parameters_MICROPHONE.SAMPLE_RATE


def unpack_to_mp4(input_filenames, output_filename):
    time_base = fractions.Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)

    readers = [hl2ss_io.create_rd(input_filename, hl2ss.ChunkSize.SINGLE_TRANSFER, None) for input_filename in input_filenames]
    [reader.open() for reader in readers]

    container = av.open(output_filename, mode='w')
    streams = [container.add_stream(get_av_codec_name(reader.port, reader.profile_ab if (reader.port == hl2ss.StreamPort.RM_DEPTH_AHAT) else reader.profile), rate=get_av_framerate(reader.port) if (get_av_framerate(reader.port) is not None) else reader.framerate) for reader in readers]
    codecs = [av.CodecContext.create(get_av_codec_name(reader.port, reader.profile_ab if (reader.port == hl2ss.StreamPort.RM_DEPTH_AHAT) else reader.profile), "r") for reader in readers]

    for stream in streams:
        stream.time_base = time_base

    base = 0

    for reader in readers:
        data = reader.get_next_packet()
        if ((data is not None) and (data.timestamp > base)):
            base = data.timestamp

    [reader.close() for reader in readers]
    [reader.open() for reader in readers]

    for reader, codec, stream in zip(readers, codecs, streams):
        while (True):
            data = reader.get_next_packet()
            if (data is None):
                break

            if (reader.port == hl2ss.StreamPort.PERSONAL_VIDEO):
                payload = hl2ss.unpack_pv(data.payload).image
            elif (reader.port == hl2ss.StreamPort.EXTENDED_VIDEO):
                payload = hl2ss.unpack_pv(data.payload).image
            else:
                payload = data.payload

            local_timestamp = data.timestamp - base
            if (local_timestamp < 0):
                continue

            for packet in codec.parse(payload):
                packet.stream = stream
                packet.pts = local_timestamp
                packet.dts = local_timestamp
                packet.time_base = time_base
                container.mux(packet)

    container.close()
    [reader.close() for reader in readers]


def unpack_to_png(input_filename, output_filename):
    rd = hl2ss_io.create_rd(input_filename, hl2ss.ChunkSize.SINGLE_TRANSFER, True)
    rd.open()

    tar = tarfile.open(output_filename, 'w')
    idx = 0

    while (True):
        data = rd.get_next_packet()
        if (data is None):
            break
        depth = cv2.imencode('.png', data.payload.depth)[1].tobytes()
        ab = cv2.imencode('.png', data.payload.ab)[1].tobytes()
        depth_info = tarfile.TarInfo(f'depth_{idx}.png')        
        ab_info = tarfile.TarInfo(f'ab_{idx}.png')
        depth_info.size = len(depth)
        ab_info.size = len(ab)
        tar.addfile(depth_info, io.BytesIO(depth))
        tar.addfile(ab_info, io.BytesIO(ab))
        idx += 1

    tar.close()
    rd.close()


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

