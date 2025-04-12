
import numpy as np
import weakref
import socket
import select
import struct
import cv2
import av


# Stream TCP Ports
class StreamPort:
    RM_VLC_LEFTFRONT     = 3800
    RM_VLC_LEFTLEFT      = 3801
    RM_VLC_RIGHTFRONT    = 3802
    RM_VLC_RIGHTRIGHT    = 3803
    RM_DEPTH_AHAT        = 3804
    RM_DEPTH_LONGTHROW   = 3805
    RM_IMU_ACCELEROMETER = 3806
    RM_IMU_GYROSCOPE     = 3807
    RM_IMU_MAGNETOMETER  = 3808
    PERSONAL_VIDEO       = 3810
    MICROPHONE           = 3811
    SPATIAL_INPUT        = 3812
    EXTENDED_EYE_TRACKER = 3817
    EXTENDED_AUDIO       = 3818
    EXTENDED_VIDEO       = 3819
    EXTENDED_DEPTH       = 3821


# IPC TCP Ports
class IPCPort:
    REMOTE_CONFIGURATION = 3809
    SPATIAL_MAPPING      = 3813
    SCENE_UNDERSTANDING  = 3814
    VOICE_INPUT          = 3815
    UNITY_MESSAGE_QUEUE  = 3816
    GUEST_MESSAGE_QUEUE  = 3820


# Default Chunk Sizes
class ChunkSize:
    RM_VLC               = 4096
    RM_DEPTH_AHAT        = 4096
    RM_DEPTH_LONGTHROW   = 4096
    RM_IMU               = 4096
    PERSONAL_VIDEO       = 4096
    MICROPHONE           = 4096
    SPATIAL_INPUT        = 4096
    EXTENDED_EYE_TRACKER = 4096
    EXTENDED_AUDIO       = 4096
    EXTENDED_VIDEO       = 4096
    EXTENDED_DEPTH       = 4096
    SINGLE_TRANSFER      = 4096


# Stream Operating Mode
# 0: Device data (e.g. video)
# 1: Device data + location data (e.g. video + camera pose)
# 2: Device constants (e.g. camera intrinsics)
# 3: Reserved
class StreamMode:
    MODE_0 = 0
    MODE_1 = 1
    MODE_2 = 2
    MODE_3 = 3


# Video Encoder Profile
#   0: H264 base
#   1: H264 main
#   2: H264 high
#   3: H265 main (HEVC)
# 255: No encoding
class VideoProfile:
    H264_BASE = 0
    H264_MAIN = 1
    H264_HIGH = 2
    H265_MAIN = 3
    RAW       = 0xFF


# Video Encoder Level
class H26xLevel:
    H264_1   =  10
    H264_1_b =  11    
    H264_1_1 =  11
    H264_1_2 =  12
    H264_1_3 =  13
    H264_2   =  20
    H264_2_1 =  21
    H264_2_2 =  22
    H264_3   =  30
    H264_3_1 =  31
    H264_3_2 =  32
    H264_4   =  40
    H264_4_1 =  41
    H264_4_2 =  42
    H264_5   =  50
    H264_5_1 =  51
    H264_5_2 =  52
    H265_1   =  30
    H265_2   =  60
    H265_2_1 =  63
    H265_3   =  90
    H265_3_1 =  93
    H265_4   = 120
    H265_4_1 = 123
    H265_5   = 150
    H265_5_1 = 153
    H265_5_2 = 156
    H265_6   = 180
    H265_6_1 = 183
    H265_6_2 = 186
    DEFAULT  = 255


# Depth Encoder Profile
# 0: Same as AB
# 1: Zdepth
class DepthProfile:
    SAME   = 0
    ZDEPTH = 1


# Audio Encoder Profile
#   0: AAC 12000 bytes/s
#   1: AAC 16000 bytes/s
#   2: AAC 20000 bytes/s
#   3: AAC 24000 bytes/s
# 255: No encoding
class AudioProfile:
    AAC_12000 = 0
    AAC_16000 = 1
    AAC_20000 = 2
    AAC_24000 = 3
    RAW       = 0xFF


# Audio Encoder Level
# 0x29: AAC Profile L2 (default)
# 0x2A: AAC Profile L4
# 0x2B: AAC Profile L5
# 0x2C: High Efficiency v1 AAC Profile L2
# 0x2E: High Efficiency v1 AAC Profile L4
# 0x2F: High Efficiency v1 AAC Profile L5
# 0x30: High Efficiency v2 AAC Profile L2
# 0x31: High Efficiency v2 AAC Profile L3
# 0x32: High Efficiency v2 AAC Profile L4
# 0x33: High Efficiency v2 AAC Profile L5
class AACLevel:
    L2      = 0x29
    L4      = 0x2A
    L5      = 0x2B
    HEV1L2  = 0x2C
    HEV1L4  = 0x2E
    HEV1L5  = 0x2F
    HEV2L2  = 0x30
    HEV2L3  = 0x31
    HEV2L4  = 0x32
    HEV2L5  = 0x33


# PNG Filters
class PNGFilterMode:
    AUTOMATIC = 0
    DISABLE   = 1
    SUB       = 2
    UP        = 3
    AVERAGE   = 4
    PAETH     = 5
    ADAPTIVE  = 6


# Encoder Properties
class H26xEncoderProperty:
    CODECAPI_AVEncCommonRateControlMode     = 0
    CODECAPI_AVEncCommonQuality             = 1
    CODECAPI_AVEncAdaptiveMode              = 2
    CODECAPI_AVEncCommonBufferSize          = 3
    CODECAPI_AVEncCommonMaxBitRate          = 4
    CODECAPI_AVEncCommonMeanBitRate         = 5
    CODECAPI_AVEncCommonQualityVsSpeed      = 6
    CODECAPI_AVEncH264CABACEnable           = 7
    CODECAPI_AVEncH264SPSID                 = 8
    CODECAPI_AVEncMPVDefaultBPictureCount   = 9
    CODECAPI_AVEncMPVGOPSize                = 10
    CODECAPI_AVEncNumWorkerThreads          = 11 
    CODECAPI_AVEncVideoContentType          = 12
    CODECAPI_AVEncVideoEncodeQP             = 13
    CODECAPI_AVEncVideoForceKeyFrame        = 14 
    CODECAPI_AVEncVideoMinQP                = 15
    CODECAPI_AVLowLatencyMode               = 16
    CODECAPI_AVEncVideoMaxQP                = 17
    CODECAPI_VideoEncoderDisplayContentType = 18
    HL2SSAPI_VideoMediaIndex                = 0xFFFFFFFFFFFFFFFB
    HL2SSAPI_VideoStrideMask                = 0xFFFFFFFFFFFFFFFC
    HL2SSAPI_AcquisitionMode                = 0xFFFFFFFFFFFFFFFD
    HL2SSAPI_VLCHostTicksOffsetConstant     = 0xFFFFFFFFFFFFFFFE
    HL2SSAPI_VLCHostTicksOffsetExposure     = 0xFFFFFFFFFFFFFFFF


# Mixed Reality Capture Hologram Rendering Perspective
# 0: Render holograms from display viewpoint
# 1: Render holograms from PV camera viewpoint
class HologramPerspective:
    DISPLAY = 0
    PV      = 1


# Audio Mixer Mode
# 0: Capture microphone audio
# 1: Capture application audio
# 2: Capture microphone and application audio
# 3: Get list of audio capture devices
class MixerMode:
    MICROPHONE = 0
    SYSTEM     = 1
    BOTH       = 2
    QUERY      = 3


# Media Category
class MediaCategory:
    Other = 0
    Communications = 1
    Media = 2
    GameChat = 3
    Speech = 4


# RM VLC Parameters
class Parameters_RM_VLC:
    WIDTH  = 640
    HEIGHT = 480
    FPS    = 30
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)
    PERIOD = 1 / FPS


# RM Depth AHAT Parameters
class Parameters_RM_DEPTH_AHAT:
    WIDTH  = 512
    HEIGHT = 512
    FPS    = 45
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)
    PERIOD = 1 / FPS


# RM Depth Long Throw Parameters
class Parameters_RM_DEPTH_LONGTHROW:
    WIDTH  = 320
    HEIGHT = 288
    FPS    = 5
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)
    PERIOD = 1 / FPS


# RM IMU Accelerometer Parameters
class Parameters_RM_IMU_ACCELEROMETER:
    BATCH_SIZE = 93


# RM IMU Gyroscope Parameters
class Parameters_RM_IMU_GYROSCOPE:
    BATCH_SIZE = 315


# RM IMU Magnetometer Parameters
class Parameters_RM_IMU_MAGNETOMETER:
    BATCH_SIZE = 11


# Microphone Parameters
class Parameters_MICROPHONE:
    CHANNELS = 2

    ARRAY_CHANNELS     = 5
    ARRAY_TOP_LEFT     = 0
    ARRAY_TOP_CENTER   = 1
    ARRAY_TOP_RIGHT    = 2
    ARRAY_BOTTOM_LEFT  = 3
    ARRAY_BOTTOM_RIGHT = 4

    SAMPLE_RATE    = 48000
    PERIOD         = 1 / SAMPLE_RATE
    GROUP_SIZE_RAW = 768
    GROUP_SIZE_AAC = 1024


# Spatial Input Parameters
class Parameters_SI:
    SAMPLE_RATE = 30
    PERIOD      = 1 / SAMPLE_RATE


# Time base for all timestamps
class TimeBase:
    HUNDREDS_OF_NANOSECONDS = 10*1000*1000
    FILETIME_EPOCH = 0
    UNIX_EPOCH = 116444736000000000


#------------------------------------------------------------------------------
# Network Client
#------------------------------------------------------------------------------

class _SIZEOF:
    CHAR     = 1
    BYTE     = 1
    SHORT    = 2
    WORD     = 2
    HALF     = 2
    INT      = 4
    DWORD    = 4
    FLOAT    = 4
    LONGLONG = 8
    QWORD    = 8
    DOUBLE   = 8


class _RANGEOF:
    U8_MAX  = 0xFF
    U16_MAX = 0xFFFF
    U32_MAX = 0xFFFFFFFF
    U64_MAX = 0xFFFFFFFFFFFFFFFF


class _client:
    def open(self, host, port, sockopt):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._f = weakref.finalize(self, lambda s : s.close(), self._socket)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, sockopt['setsockopt.IPPROTO_TCP.TCP_NODELAY'])
        self._socket.settimeout(sockopt['settimeout'])
        self._socket.connect((host, port))

    def sendall(self, data):
        self._socket.sendall(data)

    def poll(self):
        return len(select.select([self._socket], [], [], 0)[0]) > 0

    def recv(self, chunk_size):
        chunk = self._socket.recv(chunk_size)
        if (len(chunk) <= 0):
            raise Exception('connection closed')
        return chunk

    def download(self, total, chunk_size):
        data = bytearray()

        while (total > 0):
            if (chunk_size > total):
                chunk_size = total
            chunk = self.recv(chunk_size)
            data.extend(chunk)
            total -= len(chunk)

        return data

    def close(self):
        self._f.detach()
        self._socket.close()


#------------------------------------------------------------------------------
# Packet Unpacker
#------------------------------------------------------------------------------

class _packet:
    def __init__(self, timestamp, payload, pose):
        self.timestamp = timestamp
        self.payload   = payload
        self.pose      = pose


def pack_packet(packet):
    buffer = bytearray()
    buffer.extend(struct.pack('<QI', packet.timestamp, len(packet.payload)))
    buffer.extend(packet.payload)
    buffer.extend(packet.pose.tobytes() if (packet.pose is not None) else b'')
    return buffer


def unpack_packet(data):
    timestamp, payload_size = struct.unpack('<QI', data[:12])
    pose_begin = 12 + payload_size
    payload = data[12:pose_begin]
    pose = data[pose_begin:]
    return _packet(timestamp, payload, np.frombuffer(pose, dtype=np.float32).reshape((4, 4)) if (len(pose) == 64) else None)


def is_valid_pose(pose):
    return pose[3, 3] != 0


class _unpacker:
    def reset(self, mode):
        self._mode = mode
        self._state = 0
        self._buffer = bytearray()
        self._size = None
        self._packet = None
        self._pose_size = 64 if (mode == StreamMode.MODE_1) else 0

    def extend(self, chunk):
        self._buffer.extend(chunk)

    def unpack(self):        
        length = len(self._buffer)

        if ((self._state == 0) and (length >= 12)):
            self._size   = 12 + struct.unpack('<I', self._buffer[8:12])[0] + self._pose_size
            self._state  = 1

        if ((self._state == 1) and (length >= self._size)):
            self._packet = self._buffer[:self._size]
            self._buffer = self._buffer[self._size:]
            self._state  = 0
            return True

        return False

    def get(self):
        return unpack_packet(self._packet)


#------------------------------------------------------------------------------
# Packet Gatherer
#------------------------------------------------------------------------------

class _gatherer:
    def __init__(self):
        self._client = _client()
        self._unpacker = _unpacker()

    def open(self, host, port, sockopt, chunk_size, mode):
        self._chunk_size = chunk_size
        self._unpacker.reset(mode)
        self._client.open(host, port, sockopt)
        
    def sendall(self, data):
        self._client.sendall(data)

    def get_next_packet(self, wait=True):
        while (True):
            if (self._unpacker.unpack()):
                return self._unpacker.get()
            if ((not wait) and (not self._client.poll())):
                return None
            self._unpacker.extend(self._client.recv(self._chunk_size))

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Stream Configuration
#------------------------------------------------------------------------------

def _create_configuration_for_mode(mode):
    return struct.pack('<B', mode)


def _create_configuration_for_video_format(width, height, framerate):
    return struct.pack('<HHB', width, height, framerate)


def _create_configuration_for_video_divisor(divisor):
    return struct.pack('<B', divisor)


def _create_configuration_for_video_encoding(profile, level, bitrate):
    return struct.pack('<BBI', profile, level, bitrate)


def _create_configuration_for_depth_encoding(profile):
    return struct.pack('<B', profile)


def _create_configuration_for_audio_encoding(profile, level):
    return struct.pack('<BB', profile, level)


def _create_configuration_for_png_encoding(png_filter):
    return struct.pack('<I', png_filter)


def _create_configuration_for_h26x_encoding(options):
    configuration = bytearray()
    configuration.extend(struct.pack('<B', len(options)))
    for key, value in options.items():
        configuration.extend(struct.pack('<QQ', key, value))
    return bytes(configuration)


def _create_configuration_for_framerate(fps):
    return struct.pack('<B', fps)


def _create_configuration_for_mrc_video(enable, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective):
    return struct.pack('<BBBBBBBfffII', 1 if (enable) else 0, 1 if (hologram_composition) else 0, 1 if (recording_indicator) else 0, 1 if (video_stabilization) else 0, 1 if (blank_protected) else 0, 1 if (show_mesh) else 0, 1 if (shared) else 0, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective)


def _create_configuration_for_mrc_audio(mixer_mode, loopback_gain, microphone_gain):
    return struct.pack('<Iff', mixer_mode, loopback_gain, microphone_gain)


def _create_configuration_for_rm_vlc(mode, divisor, profile, level, bitrate, options):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_divisor(divisor))
    configuration.extend(_create_configuration_for_video_encoding(profile, level, bitrate))
    configuration.extend(_create_configuration_for_h26x_encoding(options))
    return bytes(configuration)


def _create_configuration_for_rm_depth_ahat(mode, divisor, profile_z, profile_ab, level, bitrate, options):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_divisor(divisor))
    configuration.extend(_create_configuration_for_depth_encoding(profile_z))
    configuration.extend(_create_configuration_for_video_encoding(profile_ab, level, bitrate))
    configuration.extend(_create_configuration_for_h26x_encoding(options))
    return bytes(configuration)


def _create_configuration_for_rm_depth_longthrow(mode, divisor, png_filter):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_divisor(divisor))
    configuration.extend(_create_configuration_for_png_encoding(png_filter))
    return bytes(configuration)


def _create_configuration_for_rm_imu(mode):
    return _create_configuration_for_mode(mode)


def _create_configuration_for_pv(mode, width, height, framerate, divisor, profile, level, bitrate, options):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_format(width, height, framerate))
    configuration.extend(_create_configuration_for_video_divisor(divisor))
    configuration.extend(_create_configuration_for_video_encoding(profile, level, bitrate))
    configuration.extend(_create_configuration_for_h26x_encoding(options))
    return bytes(configuration)


def _create_configuration_for_microphone(profile, level):
    return _create_configuration_for_audio_encoding(profile, level)


def _create_configuration_for_eet(fps):
    return _create_configuration_for_framerate(fps)


def _create_configuration_for_extended_audio(mixer_mode, loopback_gain, microphone_gain, profile, level):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mrc_audio(mixer_mode, loopback_gain, microphone_gain))
    configuration.extend(_create_configuration_for_audio_encoding(profile, level))
    return bytes(configuration)


def _create_configuration_for_extended_depth(mode, divisor, profile_z, options):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_divisor(divisor))
    configuration.extend(_create_configuration_for_depth_encoding(profile_z))
    configuration.extend(_create_configuration_for_h26x_encoding(options))
    return bytes(configuration)


def _create_configuration_for_rm_mode2(mode):
    return _create_configuration_for_mode(mode)


def _create_configuration_for_pv_mode2(mode, width, height, framerate):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_format(width, height, framerate))
    return bytes(configuration)


def extended_audio_device_mixer_mode(mixer_mode, device_index, source_index, format_index):
    return mixer_mode | (((device_index + 1) & 0x3FF) << 2) | ((source_index & 0x3FF) << 12) | ((format_index & 0x3FF) << 22)


def extended_audio_raw_configuration(media_category, shared, audio_raw, disable_effect, enable_passthrough):
    b0_2 = media_category & 7
    b3_3 = 1 if (shared) else 0
    b4_4 = 0
    b5_5 = 1 if (audio_raw) else 0
    b6_6 = 1 if (disable_effect) else 0
    b7_7 = 1 if (enable_passthrough) else 0

    return (b7_7 << 7) | (b6_6 << 6) | (b5_5 << 5) | (b4_4 << 4) | (b3_3 << 3) | b0_2


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Acquisition
#------------------------------------------------------------------------------

def _connect_client_rm_vlc(host, port, sockopt, chunk_size, mode, divisor, profile, level, bitrate, options):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_vlc(mode, divisor, profile, level, bitrate, options))
    return c


def _connect_client_rm_depth_ahat(host, port, sockopt, chunk_size, mode, divisor, profile_z, profile_ab, level, bitrate, options):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_depth_ahat(mode, divisor, profile_z, profile_ab, level, bitrate, options))
    return c


def _connect_client_rm_depth_longthrow(host, port, sockopt, chunk_size, mode, divisor, png_filter):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_depth_longthrow(mode, divisor, png_filter))
    return c


def _connect_client_rm_imu(host, port, sockopt, chunk_size, mode):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_imu(mode))
    return c


def _connect_client_pv(host, port, sockopt, chunk_size, mode, width, height, framerate, divisor, profile, level, bitrate, options):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, mode)
    c.sendall(_create_configuration_for_pv(mode, width, height, framerate, divisor, profile, level, bitrate, options))
    return c


def _connect_client_microphone(host, port, sockopt, chunk_size, profile, level):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, StreamMode.MODE_0)
    c.sendall(_create_configuration_for_microphone(profile, level))
    return c


def _connect_client_si(host, port, sockopt, chunk_size):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, StreamMode.MODE_0)
    return c


def _connect_client_eet(host, port, sockopt, chunk_size, fps):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, StreamMode.MODE_1)
    c.sendall(_create_configuration_for_eet(fps))
    return c


def _connect_client_extended_audio(host, port, sockopt, chunk_size, mixer_mode, loopback_gain, microphone_gain, profile, level):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, StreamMode.MODE_0)
    c.sendall(_create_configuration_for_extended_audio(mixer_mode, loopback_gain, microphone_gain, profile, level))
    return c


def _connect_client_extended_depth(host, port, sockopt, chunk_size, mode, divisor, profile_z, options):
    c = _gatherer()
    c.open(host, port, sockopt, chunk_size, mode)
    c.sendall(_create_configuration_for_extended_depth(mode, divisor, profile_z, options))
    return c


class _PVCNT:
    START =  0x04
    STOP   = 0x08


def start_subsystem_pv(host, port, sockopt, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective):
    c = _client()
    c.open(host, port, sockopt)
    c.sendall(_create_configuration_for_mode(_PVCNT.START | StreamMode.MODE_3))
    c.sendall(_create_configuration_for_mrc_video(enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective))
    c.close()


def stop_subsystem_pv(host, port, sockopt):
    c = _client()
    c.open(host, port, sockopt)
    c.sendall(_create_configuration_for_mode(_PVCNT.STOP | StreamMode.MODE_3))
    c.close()


#------------------------------------------------------------------------------
# Context Manager
#------------------------------------------------------------------------------

class _context_manager:
    def __enter__(self):
        self.open()
        return self
    
    def __exit__(self, *args):
        self.close()


#------------------------------------------------------------------------------
# Receiver Wrappers
#------------------------------------------------------------------------------

class rx_rm_vlc(_context_manager):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, profile, level, bitrate, options):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile = profile
        self.level = level
        self.bitrate = bitrate
        self.options = options

    def open(self):
        self._client = _connect_client_rm_vlc(self.host, self.port, self.sockopt, self.chunk, self.mode, self.divisor, self.profile, self.level, self.bitrate, self.options)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


class rx_rm_depth_ahat(_context_manager):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile_z = profile_z
        self.profile_ab = profile_ab
        self.level = level
        self.bitrate = bitrate
        self.options = options

    def open(self):
        self._client = _connect_client_rm_depth_ahat(self.host, self.port, self.sockopt, self.chunk, self.mode, self.divisor, self.profile_z, self.profile_ab, self.level, self.bitrate, self.options)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


class rx_rm_depth_longthrow(_context_manager):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, png_filter):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.png_filter = png_filter

    def open(self):
        self._client = _connect_client_rm_depth_longthrow(self.host, self.port, self.sockopt, self.chunk, self.mode, self.divisor, self.png_filter)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


class rx_rm_imu(_context_manager):
    def __init__(self, host, port, sockopt, chunk, mode):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.mode = mode

    def open(self):
        self._client = _connect_client_rm_imu(self.host, self.port, self.sockopt, self.chunk, self.mode)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


class rx_pv(_context_manager):
    def __init__(self, host, port, sockopt, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.mode = mode
        self.width = width
        self.height = height
        self.framerate = framerate
        self.divisor = divisor
        self.profile = profile
        self.level = level
        self.bitrate = bitrate
        self.options = options

    def open(self):
        self._client = _connect_client_pv(self.host, self.port, self.sockopt, self.chunk, self.mode, self.width, self.height, self.framerate, self.divisor, self.profile, self.level, self.bitrate, self.options)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


class rx_microphone(_context_manager):
    def __init__(self, host, port, sockopt, chunk, profile, level):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.profile = profile
        self.level = level

    def open(self):
        self._client = _connect_client_microphone(self.host, self.port, self.sockopt, self.chunk, self.profile, self.level)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


class rx_si(_context_manager):
    def __init__(self, host, port, sockopt, chunk):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk

    def open(self):
        self._client = _connect_client_si(self.host, self.port, self.sockopt, self.chunk)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


class rx_eet(_context_manager):
    def __init__(self, host, port, sockopt, chunk, fps):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.fps = fps

    def open(self):
        self._client = _connect_client_eet(self.host, self.port, self.sockopt, self.chunk, self.fps)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)
    
    def close(self):
        self._client.close()


class rx_extended_audio(_context_manager):
    def __init__(self, host, port, sockopt, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.mixer_mode = mixer_mode
        self.loopback_gain = loopback_gain
        self.microphone_gain = microphone_gain
        self.profile = profile
        self.level = level

    def open(self):
        self._client = _connect_client_extended_audio(self.host, self.port, self.sockopt, self.chunk, self.mixer_mode, self.loopback_gain, self.microphone_gain, self.profile, self.level)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)
    
    def close(self):
        self._client.close()


class rx_extended_depth(_context_manager):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, profile_z, options):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile_z = profile_z
        self.options = options

    def open(self):
        self._client = _connect_client_extended_depth(self.host, self.port, self.sockopt, self.chunk, self.mode, self.divisor, self.profile_z, self.options)

    def get_next_packet(self, wait=True):
        return self._client.get_next_packet(wait)

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Codecs
#------------------------------------------------------------------------------

def get_video_codec_name(profile):
    if (profile == VideoProfile.H264_BASE):
        return 'h264'
    if (profile == VideoProfile.H264_MAIN):
        return 'h264'
    if (profile == VideoProfile.H264_HIGH):
        return 'h264'
    if (profile == VideoProfile.H265_MAIN):
        return 'hevc'

    return None


def get_audio_codec_name(profile):
    if (profile == AudioProfile.AAC_12000):
        return 'aac'
    if (profile == AudioProfile.AAC_16000):
        return 'aac'
    if (profile == AudioProfile.AAC_20000):
        return 'aac'
    if (profile == AudioProfile.AAC_24000):
        return 'aac'
    
    return None


def get_audio_codec_bitrate(profile):
    if (profile == AudioProfile.AAC_12000):
        return 12000*8
    if (profile == AudioProfile.AAC_16000):
        return 16000*8
    if (profile == AudioProfile.AAC_20000):
        return 20000*8
    if (profile == AudioProfile.AAC_24000):
        return 24000*8
    
    return None


class _codec_h264:
    _aud = b'\x00\x00\x00\x01\x09\x10'

    def __init__(self):
        self._codec = self._codec = av.CodecContext.create('h264', 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload[6:] + _codec_h264._aud):
            for frame in self._codec.decode(packet):
                return frame


class _codec_hevc:
    _aud = b'\x00\x00\x00\x01\x46\x01\x03'

    def __init__(self):
        self._codec = self._codec = av.CodecContext.create('hevc', 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload + _codec_hevc._aud):
            for frame in self._codec.decode(packet):
                return frame


class _codec_aac:
    def __init__(self):
        self._codec = av.CodecContext.create('aac', 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return frame


def get_video_codec(profile):
    if (profile == VideoProfile.H264_BASE):
        return _codec_h264()
    if (profile == VideoProfile.H264_MAIN):
        return _codec_h264()
    if (profile == VideoProfile.H264_HIGH):
        return _codec_h264()
    if (profile == VideoProfile.H265_MAIN):
        return _codec_hevc()

    return None


def get_audio_codec(profile):
    if (profile == AudioProfile.AAC_12000):
        return _codec_aac()
    if (profile == AudioProfile.AAC_16000):
        return _codec_aac()
    if (profile == AudioProfile.AAC_20000):
        return _codec_aac()
    if (profile == AudioProfile.AAC_24000):
        return _codec_aac()
    
    return None


class _MetadataSize:
    RM_VLC               = 24
    RM_DEPTH_AHAT        = 8
    RM_DEPTH_LONGTHROW   = 8
    RM_IMU               = 0
    PERSONAL_VIDEO       = 80
    MICROPHONE           = 0
    SPATIAL_INPUT        = 0
    EXTENDED_EYE_TRACKER = 0
    EXTENDED_AUDIO       = 0
    EXTENDED_DEPTH       = 4

    OF = {
        StreamPort.RM_VLC_LEFTFRONT     : RM_VLC,
        StreamPort.RM_VLC_LEFTLEFT      : RM_VLC,
        StreamPort.RM_VLC_RIGHTFRONT    : RM_VLC,
        StreamPort.RM_VLC_RIGHTRIGHT    : RM_VLC,
        StreamPort.RM_DEPTH_AHAT        : RM_DEPTH_AHAT,
        StreamPort.RM_DEPTH_LONGTHROW   : RM_DEPTH_LONGTHROW,
        StreamPort.RM_IMU_ACCELEROMETER : RM_IMU,
        StreamPort.RM_IMU_GYROSCOPE     : RM_IMU,
        StreamPort.RM_IMU_MAGNETOMETER  : RM_IMU,
        StreamPort.PERSONAL_VIDEO       : PERSONAL_VIDEO,
        StreamPort.MICROPHONE           : MICROPHONE,
        StreamPort.SPATIAL_INPUT        : SPATIAL_INPUT,
        StreamPort.EXTENDED_EYE_TRACKER : EXTENDED_EYE_TRACKER,
        StreamPort.EXTENDED_AUDIO       : EXTENDED_AUDIO,
        StreamPort.EXTENDED_VIDEO       : PERSONAL_VIDEO,
        StreamPort.EXTENDED_DEPTH       : EXTENDED_DEPTH,
    }


def get_metadata_size(port):
    return _MetadataSize.OF[port]


class _decompress_zdepth:
    def __init__(self):
        import pyzdepth
        self._codec = pyzdepth.DepthCompressor()

    def decode(self, payload):
        result, width, height, decompressed = self._codec.Decompress(bytes(payload))
        return np.frombuffer(decompressed, dtype=np.uint16).reshape((height, width))


#------------------------------------------------------------------------------
# RM VLC Decoder
#------------------------------------------------------------------------------

class _RM_VLC_Frame:
    def __init__(self, image, sensor_ticks, exposure, gain):
        self.image        = image
        self.sensor_ticks = sensor_ticks
        self.exposure     = exposure
        self.gain         = gain


class _decode_rm_vlc_h26x:
    def __init__(self, profile):
        self._codec = get_video_codec(profile)

    def decode(self, payload):
        return self._codec.decode(payload).to_ndarray()[:Parameters_RM_VLC.HEIGHT, :Parameters_RM_VLC.WIDTH]


class _decode_rm_vlc_raw:
    def decode(self, payload):
        return np.frombuffer(payload, dtype=np.uint8).reshape(Parameters_RM_VLC.SHAPE)


class decode_rm_vlc:
    def __init__(self, profile):
        self._codec = _decode_rm_vlc_raw() if (profile == VideoProfile.RAW) else _decode_rm_vlc_h26x(profile)

    def decode(self, payload):
        data     = payload[:-24]
        metadata = payload[-24:]

        image        = self._codec.decode(data)
        sensor_ticks = np.frombuffer(metadata, dtype=np.uint64, offset=0,  count=1)
        exposure     = np.frombuffer(metadata, dtype=np.uint64, offset=8,  count=1)
        gain         = np.frombuffer(metadata, dtype=np.uint32, offset=16, count=1)

        return _RM_VLC_Frame(image, sensor_ticks, exposure, gain)


#------------------------------------------------------------------------------
# RM Depth Decoder
#------------------------------------------------------------------------------

class _RM_Depth_Frame:
    def __init__(self, depth, ab, sensor_ticks):
        self.depth        = depth
        self.ab           = ab
        self.sensor_ticks = sensor_ticks


class _decode_rm_depth_ahat_z_ab_h26x:
    TRUNCATE = 4

    YS = Parameters_RM_DEPTH_AHAT.HEIGHT
    CS = Parameters_RM_DEPTH_AHAT.HEIGHT // 4

    BEGIN_Z_Y = 0
    END_Z_Y   = BEGIN_Z_Y + YS
    BEGIN_I_U = END_Z_Y
    END_I_U   = BEGIN_I_U + CS
    BEGIN_I_V = END_I_U
    END_I_V   = BEGIN_I_V + CS

    def __init__(self, profile):
        self._codec = get_video_codec(profile)

    def decode(self, payload):
        yuv = self._codec.decode(payload).to_ndarray()

        y = yuv[_decode_rm_depth_ahat_z_ab_h26x.BEGIN_Z_Y : _decode_rm_depth_ahat_z_ab_h26x.END_Z_Y, :]
        u = yuv[_decode_rm_depth_ahat_z_ab_h26x.BEGIN_I_U : _decode_rm_depth_ahat_z_ab_h26x.END_I_U, :].reshape((Parameters_RM_DEPTH_AHAT.HEIGHT, -1))
        v = yuv[_decode_rm_depth_ahat_z_ab_h26x.BEGIN_I_V : _decode_rm_depth_ahat_z_ab_h26x.END_I_V, :].reshape((Parameters_RM_DEPTH_AHAT.HEIGHT, -1))

        depth = np.multiply(y, _decode_rm_depth_ahat_z_ab_h26x.TRUNCATE, dtype=np.uint16)
        ab    = np.empty((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH), dtype=np.uint16)

        u = np.square(u, dtype=np.uint16)
        v = np.square(v, dtype=np.uint16)

        ab[:, 0::4] = u
        ab[:, 1::4] = u
        ab[:, 2::4] = v
        ab[:, 3::4] = v

        return depth, ab


class _decode_rm_depth_ahat_z_ab_raw:
    _Z = 0
    _I = Parameters_RM_DEPTH_AHAT.PIXELS * _SIZEOF.WORD

    def decode(self, payload):
        depth = np.frombuffer(payload, dtype=np.uint16, offset=_decode_rm_depth_ahat_z_ab_raw._Z, count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
        ab    = np.frombuffer(payload, dtype=np.uint16, offset=_decode_rm_depth_ahat_z_ab_raw._I, count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
        
        return depth, ab


class _decode_rm_depth_ahat_x_ab_h26x:
    def __init__(self, profile):
        self._codec = get_video_codec(profile)

    def decode(self, payload):
        return np.square(self._codec.decode(payload).to_ndarray()[:Parameters_RM_DEPTH_AHAT.HEIGHT, :Parameters_RM_DEPTH_AHAT.WIDTH], dtype=np.uint16)


class _decode_rm_depth_ahat_x_ab_raw:
    def decode(self, payload):
        return np.frombuffer(payload, dtype=np.uint16, offset=0, count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)


class _decode_rm_depth_ahat:
    BASE = 8


class _decode_rm_depth_ahat_same:
    def __init__(self, profile, base):
        self._codec_f = _decode_rm_depth_ahat_z_ab_raw() if (profile == VideoProfile.RAW) else _decode_rm_depth_ahat_z_ab_h26x(profile)
        self._base = base

    def decode(self, payload):
        return self._codec_f.decode(payload[self._base:])


class _decode_rm_depth_ahat_zdepth:
    def __init__(self, profile, base):
        self._codec_z = _decompress_zdepth()
        self._codec_i = _decode_rm_depth_ahat_x_ab_raw() if (profile == VideoProfile.RAW) else _decode_rm_depth_ahat_x_ab_h26x(profile)
        self._base = base

    def decode(self, payload):
        size_z, size_i = struct.unpack_from('<II', payload, 0)

        start_z = self._base
        end_z   = start_z + size_z
        start_i = end_z
        end_i   = start_i + size_i

        depth = self._codec_z.decode(payload[start_z:end_z])
        ab    = self._codec_i.decode(payload[start_i:end_i])
        
        return depth, ab


class decode_rm_depth_ahat:
    def __init__(self, profile_z, profile_ab, base=_decode_rm_depth_ahat.BASE):
        self._codec = _decode_rm_depth_ahat_same(profile_ab, base) if (profile_z == DepthProfile.SAME) else _decode_rm_depth_ahat_zdepth(profile_ab, base)

    def decode(self, payload):
        data     = payload[:-8]
        metadata = payload[-8:]

        depth, ab    = self._codec.decode(data)
        sensor_ticks = np.frombuffer(metadata, dtype=np.uint64, offset=0, count=1)

        return _RM_Depth_Frame(depth, ab, sensor_ticks)


class _decode_rm_depth_longthrow_png:
    def decode(self, payload):
        composite = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        h, w, _   = composite.shape
        image     = composite.view(np.uint16).reshape((-1, w))
        depth     = image[:h, :]
        ab        = image[h:, :]

        return depth, ab


class _decode_rm_depth_longthrow_raw:
    _Z = 0
    _I = Parameters_RM_DEPTH_LONGTHROW.PIXELS * _SIZEOF.WORD

    def decode(self, payload):
        depth = np.frombuffer(payload, dtype=np.uint16, offset=_decode_rm_depth_longthrow_raw._Z, count=Parameters_RM_DEPTH_LONGTHROW.PIXELS).reshape(Parameters_RM_DEPTH_LONGTHROW.SHAPE)
        ab    = np.frombuffer(payload, dtype=np.uint16, offset=_decode_rm_depth_longthrow_raw._I, count=Parameters_RM_DEPTH_LONGTHROW.PIXELS).reshape(Parameters_RM_DEPTH_LONGTHROW.SHAPE)
        
        return depth, ab


class decode_rm_depth_longthrow:
    def __init__(self, profile):
        self._codec = _decode_rm_depth_longthrow_raw() if (profile == VideoProfile.RAW) else _decode_rm_depth_longthrow_png()

    def decode(self, payload):
        data     = payload[:-8]
        metadata = payload[-8:]

        depth, ab    = self._codec.decode(data)
        sensor_ticks = np.frombuffer(metadata, dtype=np.uint64, offset=0, count=1)

        return _RM_Depth_Frame(depth, ab, sensor_ticks)


#------------------------------------------------------------------------------
# RM IMU Decoder
#------------------------------------------------------------------------------

class _RM_IMU_Frame:
    def __init__(self, count, vinyl_hup_ticks, soc_ticks, x, y, z, temperature):
        self.count           = count
        self.vinyl_hup_ticks = vinyl_hup_ticks
        self.soc_ticks       = soc_ticks
        self.x               = x
        self.y               = y
        self.z               = z
        self.temperature     = temperature


def rm_imu_fix_soc_ticks(vinyl_hup_ticks, soc_ticks):
    return soc_ticks + ((vinyl_hup_ticks - vinyl_hup_ticks[0]) // 100)


class decode_rm_imu:
    def decode(self, payload):
        data_u8  = np.frombuffer(payload, dtype=np.uint8)
        data_u64 = data_u8.view(np.uint64)
        data_f32 = data_u8.view(np.float32)

        count           = len(payload) // 32
        vinyl_hup_ticks = data_u64[( 0 // 8)::(32 // 8)]
        soc_ticks       = data_u64[( 8 // 8)::(32 // 8)]
        x               = data_f32[(16 // 4)::(32 // 4)]
        y               = data_f32[(20 // 4)::(32 // 4)]
        z               = data_f32[(24 // 4)::(32 // 4)]
        temperature     = data_f32[(28 // 4)::(32 // 4)]
        patch_soc_ticks = rm_imu_fix_soc_ticks(vinyl_hup_ticks, soc_ticks) if (soc_ticks[0] == soc_ticks[-1]) else soc_ticks

        return _RM_IMU_Frame(count, vinyl_hup_ticks, patch_soc_ticks, x, y, z, temperature)


def rm_imu_get_batch_size(port):
    if (port == StreamPort.RM_IMU_ACCELEROMETER):
        return Parameters_RM_IMU_ACCELEROMETER.BATCH_SIZE
    if (port == StreamPort.RM_IMU_GYROSCOPE):
        return Parameters_RM_IMU_GYROSCOPE.BATCH_SIZE
    if (port == StreamPort.RM_IMU_MAGNETOMETER):
        return Parameters_RM_IMU_MAGNETOMETER.BATCH_SIZE


#------------------------------------------------------------------------------
# PV Decoder
#------------------------------------------------------------------------------

class PV_FocusState:
    UNINITIALIZED = 0
    LOST          = 1
    SEARCHING     = 2
    FOCUSED       = 3
    FAILED        = 4


class _PV_Frame:
    def __init__(self, image, focal_length, principal_point, exposure_time, exposure_compensation, lens_position, focus_state, iso_speed, white_balance, iso_gains, white_balance_gains, resolution):
        self.image                 = image
        self.focal_length          = focal_length
        self.principal_point       = principal_point
        self.exposure_time         = exposure_time
        self.exposure_compensation = exposure_compensation
        self.lens_position         = lens_position
        self.focus_state           = focus_state
        self.iso_speed             = iso_speed
        self.white_balance         = white_balance
        self.iso_gains             = iso_gains
        self.white_balance_gains   = white_balance_gains
        self.resolution            = resolution


def pv_get_video_stride(width):
    return (width + 63) & ~63


class _decode_pv_h26x:
    def __init__(self, profile):
        self._codec = get_video_codec(profile)

    def decode(self, payload, width, height, format):
        return self._codec.decode(payload).to_ndarray(format=format)


class _decode_pv_raw:
    _cv2_nv12_format = {
        'rgb24' : cv2.COLOR_YUV2RGB_NV12,
        'bgr24' : cv2.COLOR_YUV2BGR_NV12,
        'rgba'  : cv2.COLOR_YUV2RGBA_NV12,
        'bgra'  : cv2.COLOR_YUV2BGRA_NV12,
        'gray8' : cv2.COLOR_YUV2GRAY_NV12,
        'nv12'  : None
    }

    def decode(self, payload, width, height, format):
        image = np.frombuffer(payload, dtype=np.uint8)
        if (format == 'any'):
            return image
        image = image.reshape(((height * 3) // 2, -1))[:, :width]
        sf = _decode_pv_raw._cv2_nv12_format[format]
        return image if (sf is None) else cv2.cvtColor(image, sf)


class decode_pv:
    def __init__(self, profile):
        self._codec =  _decode_pv_raw() if (profile == VideoProfile.RAW) else _decode_pv_h26x(profile)

    def decode(self, payload, format):
        data     = payload[:-80]
        metadata = payload[-80:]

        focal_length          = np.frombuffer(metadata, dtype=np.float32, offset=0,  count=2)
        principal_point       = np.frombuffer(metadata, dtype=np.float32, offset=8,  count=2)
        exposure_time         = np.frombuffer(metadata, dtype=np.uint64,  offset=16, count=1)
        exposure_compensation = np.frombuffer(metadata, dtype=np.uint64,  offset=24, count=2)
        lens_position         = np.frombuffer(metadata, dtype=np.uint32,  offset=40, count=1)
        focus_state           = np.frombuffer(metadata, dtype=np.uint32,  offset=44, count=1)
        iso_speed             = np.frombuffer(metadata, dtype=np.uint32,  offset=48, count=1)
        white_balance         = np.frombuffer(metadata, dtype=np.uint32,  offset=52, count=1)
        iso_gains             = np.frombuffer(metadata, dtype=np.float32, offset=56, count=2)
        white_balance_gains   = np.frombuffer(metadata, dtype=np.float32, offset=64, count=3)
        resolution            = np.frombuffer(metadata, dtype=np.uint16,  offset=76, count=2)
        image                 = self._codec.decode(data, resolution[0], resolution[1], format)

        return _PV_Frame(image, focal_length, principal_point, exposure_time, exposure_compensation, lens_position, focus_state, iso_speed, white_balance, iso_gains, white_balance_gains, resolution)


#------------------------------------------------------------------------------
# Microphone Decoder
#------------------------------------------------------------------------------

def microphone_parameters(profile, level):
    return (np.float32, (2, -1)) if (profile != AudioProfile.RAW) else (np.float32, (1, -1)) if (level == AACLevel.L5) else (np.int16, (1, -1))


class _decode_microphone_aac:
    def __init__(self, profile):
        self._codec = get_audio_codec(profile)

    def decode(self, payload):
        return self._codec.decode(payload).to_ndarray()


class _decode_microphone_raw:
    def __init__(self, level):
        self.dtype, self.shape = microphone_parameters(AudioProfile.RAW, level)

    def decode(self, payload):
        return np.frombuffer(payload, dtype=self.dtype).reshape(self.shape)


class decode_microphone:
    def __init__(self, profile, level):
        self._codec = _decode_microphone_raw(level) if (profile == AudioProfile.RAW) else _decode_microphone_aac(profile)
    
    def decode(self, payload):
        return self._codec.decode(payload)


def microphone_planar_to_packed(array, channels):
    data = np.zeros((1, array.size), dtype=array.dtype)
    for i in range(0, channels):
        data[0, i::channels] = array[i, :]
    return data


def microphone_packed_to_planar(array, channels):
    data = np.zeros((channels, array.size // channels), dtype=array.dtype)
    for i in range(0, channels):
        data[i, :] = array[0, i::channels]
    return data


#------------------------------------------------------------------------------
# SI Decoder
#------------------------------------------------------------------------------

class SI_HandJointKind:
    Palm = 0
    Wrist = 1
    ThumbMetacarpal = 2
    ThumbProximal = 3
    ThumbDistal = 4
    ThumbTip = 5
    IndexMetacarpal = 6
    IndexProximal = 7
    IndexIntermediate = 8
    IndexDistal = 9
    IndexTip = 10
    MiddleMetacarpal = 11
    MiddleProximal = 12
    MiddleIntermediate = 13
    MiddleDistal = 14
    MiddleTip = 15
    RingMetacarpal = 16
    RingProximal = 17
    RingIntermediate = 18
    RingDistal = 19
    RingTip = 20
    LittleMetacarpal = 21
    LittleProximal = 22
    LittleIntermediate = 23
    LittleDistal = 24
    LittleTip = 25
    TOTAL = 26


class _SI_HeadPose:
    def __init__(self, position, forward, up):
        self.position = position
        self.forward  = forward
        self.up       = up


class _SI_EyeRay:
    def __init__(self, origin, direction):
        self.origin    = origin
        self.direction = direction


class _SI_HandPose:
    def __init__(self, orientation, position, radius, accuracy):
        self.orientation = orientation
        self.position    = position
        self.radius      = radius
        self.accuracy    = accuracy


class _SI_Frame:
    def __init__(self, head_pose, eye_ray, hand_left, hand_right, head_pose_valid, eye_ray_valid, hand_left_valid, hand_right_valid):
        self.head_pose        = head_pose
        self.eye_ray          = eye_ray
        self.hand_left        = hand_left
        self.hand_right       = hand_right
        self.head_pose_valid  = head_pose_valid
        self.eye_ray_valid    = eye_ray_valid
        self.hand_left_valid  = hand_left_valid
        self.hand_right_valid = hand_right_valid


class decode_si:
    def decode(self, payload):
        status = payload[:4]
        data   = payload[4:]

        valid = struct.unpack('<I', status)[0]
        f     = np.frombuffer(data, dtype=np.float32)

        head_pose         = _SI_HeadPose(f[0:3], f[3:6], f[6:9])
        eye_ray           = _SI_EyeRay(f[9:12], f[12:15])
        hands             = f[15:].reshape((-1, 9))
        hands_orientation = hands[:, 0:4]
        hands_position    = hands[:, 4:7]
        hands_radius      = hands[:, 7]
        hands_accuracy    = hands[:, 8].view(np.int32)
        hand_left         = _SI_HandPose(hands_orientation[:SI_HandJointKind.TOTAL, :], hands_position[:SI_HandJointKind.TOTAL, :], hands_radius[:SI_HandJointKind.TOTAL], hands_accuracy[:SI_HandJointKind.TOTAL])
        hand_right        = _SI_HandPose(hands_orientation[SI_HandJointKind.TOTAL:, :], hands_position[SI_HandJointKind.TOTAL:, :], hands_radius[SI_HandJointKind.TOTAL:], hands_accuracy[SI_HandJointKind.TOTAL:])
        head_pose_valid   = (valid & 0x01) != 0
        eye_ray_valid     = (valid & 0x02) != 0
        hand_left_valid   = (valid & 0x04) != 0
        hand_right_valid  = (valid & 0x08) != 0

        return _SI_Frame(head_pose, eye_ray, hand_left, hand_right, head_pose_valid, eye_ray_valid, hand_left_valid, hand_right_valid)


class _SI_JointName:
    OF = [
        'Palm',
        'Wrist',
        'ThumbMetacarpal',
        'ThumbProximal',
        'ThumbDistal',
        'ThumbTip',
        'IndexMetacarpal',
        'IndexProximal',
        'IndexIntermediate',
        'IndexDistal',
        'IndexTip',
        'MiddleMetacarpal',
        'MiddleProximal',
        'MiddleIntermediate',
        'MiddleDistal',
        'MiddleTip',
        'RingMetacarpal',
        'RingProximal',
        'RingIntermediate',
        'RingDistal',
        'RingTip',
        'LittleMetacarpal',
        'LittleProximal',
        'LittleIntermediate',
        'LittleDistal',
        'LittleTip',
    ]


def si_get_joint_name(joint_kind):
    return _SI_JointName.OF[joint_kind]


#------------------------------------------------------------------------------
# EET Decoder
#------------------------------------------------------------------------------

class _EET_Frame:
    def __init__(self, combined_ray, left_ray, right_ray, left_openness, right_openness, vergence_distance, calibration_valid, combined_ray_valid, left_ray_valid, right_ray_valid, left_openness_valid, right_openness_valid, vergence_distance_valid):
        self.combined_ray            = combined_ray
        self.left_ray                = left_ray
        self.right_ray               = right_ray
        self.left_openness           = left_openness
        self.right_openness          = right_openness
        self.vergence_distance       = vergence_distance
        self.calibration_valid       = calibration_valid
        self.combined_ray_valid      = combined_ray_valid
        self.left_ray_valid          = left_ray_valid
        self.right_ray_valid         = right_ray_valid
        self.left_openness_valid     = left_openness_valid
        self.right_openness_valid    = right_openness_valid
        self.vergence_distance_valid = vergence_distance_valid


class decode_eet:
    def decode(self, payload):
        reserved = payload[:4]
        data     = payload[4:-4]
        status   = payload[-4:]

        f     = np.frombuffer(data, dtype=np.float32)
        valid = struct.unpack('<I', status)[0]

        combined_ray            = _SI_EyeRay(f[ 0: 3], f[ 3: 6])
        left_ray                = _SI_EyeRay(f[ 6: 9], f[ 9:12])
        right_ray               = _SI_EyeRay(f[12:15], f[15:18])
        left_openness           = f[18]
        right_openness          = f[19]
        vergence_distance       = f[20]
        calibration_valid       = (valid & 0x01) != 0
        combined_ray_valid      = (valid & 0x02) != 0
        left_ray_valid          = (valid & 0x04) != 0
        right_ray_valid         = (valid & 0x08) != 0
        left_openness_valid     = (valid & 0x10) != 0
        right_openness_valid    = (valid & 0x20) != 0
        vergence_distance_valid = (valid & 0x40) != 0

        return _EET_Frame(combined_ray, left_ray, right_ray, left_openness, right_openness, vergence_distance, calibration_valid, combined_ray_valid, left_ray_valid, right_ray_valid, left_openness_valid, right_openness_valid, vergence_distance_valid)


#------------------------------------------------------------------------------
# Extended Audio Decoder
#------------------------------------------------------------------------------

def extended_audio_parameters(profile, level):
    return (np.float32, (2, -1)) if (profile != AudioProfile.RAW) else (np.int8, (1, -1)) if ((level & 0x80) != 0) else (np.int16, (1, -1))


class _decode_extended_audio_aac:
    def __init__(self, profile):
        self._codec = get_audio_codec(profile)

    def decode(self, payload):
        return self._codec.decode(payload).to_ndarray()


class _decode_extended_audio_raw:
    def __init__(self, level):
        self.dtype, self.shape = extended_audio_parameters(AudioProfile.RAW, level)

    def decode(self, payload):
        return np.frombuffer(payload, dtype=self.dtype).reshape(self.shape)


class decode_extended_audio:
    def __init__(self, profile, level):
        self._codec = _decode_extended_audio_raw(level) if (profile == AudioProfile.RAW) else _decode_extended_audio_aac(profile)
    
    def decode(self, payload):
        return self._codec.decode(payload)


#------------------------------------------------------------------------------
# Extended Depth Decoder
#------------------------------------------------------------------------------

class _EZ_Frame:
    def __init__(self, depth, resolution):
        self.depth      = depth
        self.resolution = resolution


class _decode_extended_depth_zdepth:
    def __init__(self):
        self._codec = _decompress_zdepth()

    def decode(self, payload, width, height):
        return self._codec.decode(payload)


class _decode_extended_depth_raw:
    def decode(self, payload, width, height):
        return np.frombuffer(payload, dtype=np.uint16).reshape((height, width))


class decode_extended_depth:
    def __init__(self, profile_z):
        self._codec = _decode_extended_depth_zdepth() if (profile_z == DepthProfile.ZDEPTH) else _decode_extended_depth_raw()

    def decode(self, payload):
        data     = payload[:-4]
        metadata = payload[-4:]

        resolution = np.frombuffer(metadata, dtype=np.uint16, offset=0, count=2)
        depth      = self._codec.decode(data, resolution[0], resolution[1])

        return _EZ_Frame(depth, resolution)


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_rm_vlc(rx_rm_vlc):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, profile, level, bitrate, options):
        super().__init__(host, port, sockopt, chunk, mode, divisor, profile, level, bitrate, options)

    def open(self):
        self._codec = decode_rm_vlc(self.profile)
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_rm_depth_ahat(rx_rm_depth_ahat):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options):
        super().__init__(host, port, sockopt, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)
        
    def open(self):
        self._codec = decode_rm_depth_ahat(self.profile_z, self.profile_ab)
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_rm_depth_longthrow(rx_rm_depth_longthrow):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, png_filter):
        super().__init__(host, port, sockopt, chunk, mode, divisor, png_filter)

    def open(self):
        self._codec = decode_rm_depth_longthrow(self.png_filter)
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_rm_imu(rx_rm_imu):
    def __init__(self, host, port, sockopt, chunk, mode):
        super().__init__(host, port, sockopt, chunk, mode)

    def open(self):
        self._codec = decode_rm_imu()
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data
    
    def close(self):
        super().close()


class rx_decoded_pv(rx_pv):
    def __init__(self, host, port, sockopt, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options, format):
        super().__init__(host, port, sockopt, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)
        self.format = format
        
    def open(self):        
        self._codec = decode_pv(self.profile)
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload, self.format)
        return data

    def close(self):
        super().close()


class rx_decoded_microphone(rx_microphone):
    def __init__(self, host, port, sockopt, chunk, profile, level):
        super().__init__(host, port, sockopt, chunk, profile, level)
        
    def open(self):
        self._codec = decode_microphone(self.profile, self.level)
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_si(rx_si):
    def __init__(self, host, port, sockopt, chunk):
        super().__init__(host, port, sockopt, chunk)

    def open(self):
        self._codec = decode_si()
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data
    
    def close(self):
        super().close()


class rx_decoded_eet(rx_eet):
    def __init__(self, host, port, sockopt, chunk, fps):
        super().__init__(host, port, sockopt, chunk, fps)

    def open(self):
        self._codec = decode_eet()
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data
    
    def close(self):
        super().close()


class rx_decoded_extended_audio(rx_extended_audio):
    def __init__(self, host, port, sockopt, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level):
        super().__init__(host, port, sockopt, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level)
    
    def open(self):
        self._codec = decode_extended_audio(self.profile, self.level)
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_extended_depth(rx_extended_depth):
    def __init__(self, host, port, sockopt, chunk, mode, divisor, profile_z, options):
        super().__init__(host, port, sockopt, chunk, mode, divisor, profile_z, options)

    def open(self):
        self._codec = decode_extended_depth(self.profile_z)
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data
    
    def close(self):
        super().close()


#------------------------------------------------------------------------------
# Mode 2 Data Acquisition
#------------------------------------------------------------------------------

class _Mode2Layout_RM_VLC:
    BEGIN_UV2X       = 0
    END_UV2X         = BEGIN_UV2X + Parameters_RM_VLC.PIXELS
    BEGIN_UV2Y       = END_UV2X
    END_UV2Y         = BEGIN_UV2Y + Parameters_RM_VLC.PIXELS
    BEGIN_EXTRINSICS = END_UV2Y
    END_EXTRINSICS   = BEGIN_EXTRINSICS + 16
    BEGIN_MAPX       = END_EXTRINSICS
    END_MAPX         = BEGIN_MAPX + Parameters_RM_VLC.PIXELS
    BEGIN_MAPY       = END_MAPX
    END_MAPY         = BEGIN_MAPY + Parameters_RM_VLC.PIXELS
    BEGIN_K          = END_MAPY
    END_K            = BEGIN_K + 4
    FLOAT_COUNT      = 4*Parameters_RM_VLC.PIXELS + 16 + 4


class _Mode2Layout_RM_DEPTH_AHAT:
    BEGIN_UV2X       = 0
    END_UV2X         = BEGIN_UV2X + Parameters_RM_DEPTH_AHAT.PIXELS
    BEGIN_UV2Y       = END_UV2X
    END_UV2Y         = BEGIN_UV2Y + Parameters_RM_DEPTH_AHAT.PIXELS
    BEGIN_EXTRINSICS = END_UV2Y
    END_EXTRINSICS   = BEGIN_EXTRINSICS + 16
    BEGIN_SCALE      = END_EXTRINSICS
    END_SCALE        = BEGIN_SCALE + 1
    BEGIN_ALIAS      = END_SCALE
    END_ALIAS        = BEGIN_ALIAS + 1
    BEGIN_MAPX       = END_ALIAS
    END_MAPX         = BEGIN_MAPX + Parameters_RM_DEPTH_AHAT.PIXELS
    BEGIN_MAPY       = END_MAPX
    END_MAPY         = BEGIN_MAPY + Parameters_RM_DEPTH_AHAT.PIXELS
    BEGIN_K          = END_MAPY
    END_K            = BEGIN_K + 4
    FLOAT_COUNT      = 4*Parameters_RM_DEPTH_AHAT.PIXELS + 16 + 1 + 1 + 4


class _Mode2Layout_RM_DEPTH_LONGTHROW:
    BEGIN_UV2X       = 0
    END_UV2X         = BEGIN_UV2X + Parameters_RM_DEPTH_LONGTHROW.PIXELS
    BEGIN_UV2Y       = END_UV2X
    END_UV2Y         = BEGIN_UV2Y + Parameters_RM_DEPTH_LONGTHROW.PIXELS
    BEGIN_EXTRINSICS = END_UV2Y
    END_EXTRINSICS   = BEGIN_EXTRINSICS + 16
    BEGIN_SCALE      = END_EXTRINSICS
    END_SCALE        = BEGIN_SCALE + 1
    BEGIN_MAPX       = END_SCALE
    END_MAPX         = BEGIN_MAPX + Parameters_RM_DEPTH_LONGTHROW.PIXELS
    BEGIN_MAPY       = END_MAPX
    END_MAPY         = BEGIN_MAPY + Parameters_RM_DEPTH_LONGTHROW.PIXELS
    BEGIN_K          = END_MAPY
    END_K            = BEGIN_K + 4
    FLOAT_COUNT      = 4*Parameters_RM_DEPTH_LONGTHROW.PIXELS + 16 + 1 + 4


class _Mode2Layout_RM_IMU:
    BEGIN_EXTRINSICS = 0
    END_EXTRINSICS   = BEGIN_EXTRINSICS + 16
    FLOAT_COUNT      = 16


class _Mode2Layout_PV:
    BEGIN_FOCALLENGTH          = 0
    END_FOCALLENGTH            = BEGIN_FOCALLENGTH + 2
    BEGIN_PRINCIPALPOINT       = END_FOCALLENGTH
    END_PRINCIPAL_POINT        = BEGIN_PRINCIPALPOINT + 2
    BEGIN_RADIALDISTORTION     = END_PRINCIPAL_POINT
    END_RADIALDISTORTION       = BEGIN_RADIALDISTORTION + 3
    BEGIN_TANGENTIALDISTORTION = END_RADIALDISTORTION
    END_TANGENTIALDISTORTION   = BEGIN_TANGENTIALDISTORTION + 2
    BEGIN_PROJECTION           = END_TANGENTIALDISTORTION
    END_PROJECTION             = BEGIN_PROJECTION + 16
    BEGIN_EXTRINSICS           = END_PROJECTION
    END_EXTRINSICS             = BEGIN_EXTRINSICS + 16
    BEGIN_INTRINSICS_MF        = END_EXTRINSICS
    END_INTRINSICS_MF          = BEGIN_INTRINSICS_MF + 4
    BEGIN_EXTRINSICS_MF        = END_INTRINSICS_MF
    END_EXTRINSICS_MF          = BEGIN_EXTRINSICS_MF + 7
    FLOAT_COUNT                = 2 + 2 + 3 + 2 + 16 + 16 + 4 + 7


class _Mode2_RM_VLC:
    def __init__(self, uv2xy, extrinsics, undistort_map, intrinsics):
        self.uv2xy         = uv2xy
        self.extrinsics    = extrinsics
        self.undistort_map = undistort_map
        self.intrinsics    = intrinsics


class _Mode2_RM_DEPTH_AHAT:
    def __init__(self, uv2xy, extrinsics, scale, alias, undistort_map, intrinsics):
        self.uv2xy         = uv2xy
        self.extrinsics    = extrinsics
        self.scale         = scale
        self.alias         = alias
        self.undistort_map = undistort_map
        self.intrinsics    = intrinsics


class _Mode2_RM_DEPTH_LONGTHROW:
    def __init__(self, uv2xy, extrinsics, scale, undistort_map, intrinsics):
        self.uv2xy         = uv2xy
        self.extrinsics    = extrinsics
        self.scale         = scale
        self.undistort_map = undistort_map
        self.intrinsics    = intrinsics


class _Mode2_RM_IMU:
    def __init__(self, extrinsics):
        self.extrinsics = extrinsics


class _Mode2_PV:
    def __init__(self, focal_length, principal_point, radial_distortion, tangential_distortion, projection, intrinsics, extrinsics, intriniscs_mf, extrinsics_mf):
        self.focal_length          = focal_length
        self.principal_point       = principal_point
        self.radial_distortion     = radial_distortion
        self.tangential_distortion = tangential_distortion
        self.projection            = projection
        self.intrinsics            = intrinsics
        self.extrinsics            = extrinsics
        self.intrinsics_mf         = intriniscs_mf
        self.extrinsics_mf         = extrinsics_mf


def _download_mode2_data(host, port, sockopt, configuration, bytes):
    c = _client()

    c.open(host, port, sockopt)
    c.sendall(configuration)
    data = c.download(bytes, ChunkSize.SINGLE_TRANSFER)
    c.close()

    return data


def download_calibration_rm_vlc(host, port, sockopt):
    data   = _download_mode2_data(host, port, sockopt, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_VLC.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    uv2x       = floats[_Mode2Layout_RM_VLC.BEGIN_UV2X       : _Mode2Layout_RM_VLC.END_UV2X      ].reshape(Parameters_RM_VLC.SHAPE)
    uv2y       = floats[_Mode2Layout_RM_VLC.BEGIN_UV2Y       : _Mode2Layout_RM_VLC.END_UV2Y      ].reshape(Parameters_RM_VLC.SHAPE)
    extrinsics = floats[_Mode2Layout_RM_VLC.BEGIN_EXTRINSICS : _Mode2Layout_RM_VLC.END_EXTRINSICS].reshape((4, 4))
    mapx       = floats[_Mode2Layout_RM_VLC.BEGIN_MAPX       : _Mode2Layout_RM_VLC.END_MAPX      ].reshape(Parameters_RM_VLC.SHAPE)
    mapy       = floats[_Mode2Layout_RM_VLC.BEGIN_MAPY       : _Mode2Layout_RM_VLC.END_MAPY      ].reshape(Parameters_RM_VLC.SHAPE)
    k          = floats[_Mode2Layout_RM_VLC.BEGIN_K          : _Mode2Layout_RM_VLC.END_K         ]

    intrinsics = np.array([[k[0], 0, 0, 0], [0, k[1], 0, 0], [k[2], k[3], 1, 0], [0, 0, 0, 1]], dtype=np.float32)
    
    return _Mode2_RM_VLC(np.dstack((uv2x, uv2y)), extrinsics, np.dstack((mapx, mapy)), intrinsics)


def download_calibration_rm_depth_ahat(host, port, sockopt):
    data   = _download_mode2_data(host, port, sockopt, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_DEPTH_AHAT.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    uv2x       = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_UV2X       : _Mode2Layout_RM_DEPTH_AHAT.END_UV2X      ].reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
    uv2y       = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_UV2Y       : _Mode2Layout_RM_DEPTH_AHAT.END_UV2Y      ].reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
    extrinsics = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_EXTRINSICS : _Mode2Layout_RM_DEPTH_AHAT.END_EXTRINSICS].reshape((4, 4))
    scale      = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_SCALE      : _Mode2Layout_RM_DEPTH_AHAT.END_SCALE     ]
    alias      = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_ALIAS      : _Mode2Layout_RM_DEPTH_AHAT.END_ALIAS     ]
    mapx       = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_MAPX       : _Mode2Layout_RM_DEPTH_AHAT.END_MAPX      ].reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
    mapy       = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_MAPY       : _Mode2Layout_RM_DEPTH_AHAT.END_MAPY      ].reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
    k          = floats[_Mode2Layout_RM_DEPTH_AHAT.BEGIN_K          : _Mode2Layout_RM_DEPTH_AHAT.END_K         ]

    intrinsics = np.array([[k[0], 0, 0, 0], [0, k[1], 0, 0], [k[2], k[3], 1, 0], [0, 0, 0, 1]], dtype=np.float32)

    return _Mode2_RM_DEPTH_AHAT(np.dstack((uv2x, uv2y)), extrinsics, scale, alias, np.dstack((mapx, mapy)), intrinsics)


def download_calibration_rm_depth_longthrow(host, port, sockopt):
    data   = _download_mode2_data(host, port, sockopt, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_DEPTH_LONGTHROW.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    uv2x       = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_UV2X       : _Mode2Layout_RM_DEPTH_LONGTHROW.END_UV2X      ].reshape(Parameters_RM_DEPTH_LONGTHROW.SHAPE)
    uv2y       = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_UV2Y       : _Mode2Layout_RM_DEPTH_LONGTHROW.END_UV2Y      ].reshape(Parameters_RM_DEPTH_LONGTHROW.SHAPE)
    extrinsics = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_EXTRINSICS : _Mode2Layout_RM_DEPTH_LONGTHROW.END_EXTRINSICS].reshape((4, 4))
    scale      = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_SCALE      : _Mode2Layout_RM_DEPTH_LONGTHROW.END_SCALE     ]
    mapx       = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_MAPX       : _Mode2Layout_RM_DEPTH_LONGTHROW.END_MAPX      ].reshape(Parameters_RM_DEPTH_LONGTHROW.SHAPE)
    mapy       = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_MAPY       : _Mode2Layout_RM_DEPTH_LONGTHROW.END_MAPY      ].reshape(Parameters_RM_DEPTH_LONGTHROW.SHAPE)
    k          = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_K          : _Mode2Layout_RM_DEPTH_LONGTHROW.END_K         ]

    intrinsics = np.array([[k[0], 0, 0, 0], [0, k[1], 0, 0], [k[2], k[3], 1, 0], [0, 0, 0, 1]], dtype=np.float32)

    return _Mode2_RM_DEPTH_LONGTHROW(np.dstack((uv2x, uv2y)), extrinsics, scale, np.dstack((mapx, mapy)), intrinsics)


def download_calibration_rm_imu(host, port, sockopt):
    data   = _download_mode2_data(host, port, sockopt, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_IMU.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    extrinsics = floats[_Mode2Layout_RM_IMU.BEGIN_EXTRINSICS : _Mode2Layout_RM_IMU.END_EXTRINSICS].reshape((4, 4))

    return _Mode2_RM_IMU(extrinsics)


def download_calibration_pv(host, port, sockopt, width, height, framerate):
    data   = _download_mode2_data(host, port, sockopt, _create_configuration_for_pv_mode2(StreamMode.MODE_2, width, height, framerate), _Mode2Layout_PV.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    focal_length          = floats[_Mode2Layout_PV.BEGIN_FOCALLENGTH          : _Mode2Layout_PV.END_FOCALLENGTH         ]
    principal_point       = floats[_Mode2Layout_PV.BEGIN_PRINCIPALPOINT       : _Mode2Layout_PV.END_PRINCIPAL_POINT     ]
    radial_distortion     = floats[_Mode2Layout_PV.BEGIN_RADIALDISTORTION     : _Mode2Layout_PV.END_RADIALDISTORTION    ]
    tangential_distortion = floats[_Mode2Layout_PV.BEGIN_TANGENTIALDISTORTION : _Mode2Layout_PV.END_TANGENTIALDISTORTION]
    projection            = floats[_Mode2Layout_PV.BEGIN_PROJECTION           : _Mode2Layout_PV.END_PROJECTION          ].reshape((4, 4))
    extrinsics            = floats[_Mode2Layout_PV.BEGIN_EXTRINSICS           : _Mode2Layout_PV.END_EXTRINSICS          ].reshape((4, 4))
    intrinsics_mf         = floats[_Mode2Layout_PV.BEGIN_INTRINSICS_MF        : _Mode2Layout_PV.END_INTRINSICS_MF       ]
    extrinsics_mf         = floats[_Mode2Layout_PV.BEGIN_EXTRINSICS_MF        : _Mode2Layout_PV.END_EXTRINSICS_MF       ]

    intrinsics = np.array([[-focal_length[0], 0, 0, 0], [0, focal_length[1], 0, 0], [principal_point[0], principal_point[1], 1, 0], [0, 0, 0, 1]], dtype=np.float32)

    return _Mode2_PV(focal_length, principal_point, radial_distortion, tangential_distortion, projection, intrinsics, extrinsics, intrinsics_mf, extrinsics_mf)


def download_devicelist_extended_audio(host, port, sockopt, profile, level):
    c = _client()
    c.open(host, port, sockopt)
    c.sendall(_create_configuration_for_extended_audio(MixerMode.QUERY, 1.0, 1.0, profile, level))
    size = struct.unpack('<I', c.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))[0]
    query = c.download(size, ChunkSize.SINGLE_TRANSFER).decode('utf-16')
    c.close()
    return query


def download_devicelist_extended_video(host, port, sockopt):
    c = _client()
    c.open(host, port, sockopt)
    c.sendall(_create_configuration_for_mode(StreamMode.MODE_2))
    size = struct.unpack('<I', c.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))[0]
    query = c.download(size, ChunkSize.SINGLE_TRANSFER).decode('utf-16')
    c.close()
    return query


#------------------------------------------------------------------------------
# Port Information
#------------------------------------------------------------------------------

class _PortName:
    OF = {
        StreamPort.RM_VLC_LEFTFRONT     : 'rm_vlc_leftfront',
        StreamPort.RM_VLC_LEFTLEFT      : 'rm_vlc_leftleft',
        StreamPort.RM_VLC_RIGHTFRONT    : 'rm_vlc_rightfront', 
        StreamPort.RM_VLC_RIGHTRIGHT    : 'rm_vlc_rightright', 
        StreamPort.RM_DEPTH_AHAT        : 'rm_depth_ahat', 
        StreamPort.RM_DEPTH_LONGTHROW   : 'rm_depth_longthrow', 
        StreamPort.RM_IMU_ACCELEROMETER : 'rm_imu_accelerometer', 
        StreamPort.RM_IMU_GYROSCOPE     : 'rm_imu_gyroscope', 
        StreamPort.RM_IMU_MAGNETOMETER  : 'rm_imu_magnetometer', 
        IPCPort.REMOTE_CONFIGURATION    : 'remote_configuration', 
        StreamPort.PERSONAL_VIDEO       : 'personal_video', 
        StreamPort.MICROPHONE           : 'microphone', 
        StreamPort.SPATIAL_INPUT        : 'spatial_input', 
        IPCPort.SPATIAL_MAPPING         : 'spatial_mapping', 
        IPCPort.SCENE_UNDERSTANDING     : 'scene_understanding',
        IPCPort.VOICE_INPUT             : 'voice_input',
        IPCPort.UNITY_MESSAGE_QUEUE     : 'unity_message_queue',
        StreamPort.EXTENDED_EYE_TRACKER : 'extended_eye_tracker',
        StreamPort.EXTENDED_AUDIO       : 'extended_audio',
        StreamPort.EXTENDED_VIDEO       : 'extended_video',
        IPCPort.GUEST_MESSAGE_QUEUE     : 'guest_message_queue',
        StreamPort.EXTENDED_DEPTH       : 'extended_depth',
    }


def get_port_name(port):
    return _PortName.OF[port]


#------------------------------------------------------------------------------
# Remote Configuration
#------------------------------------------------------------------------------

class HS_MarkerState:
    Disable = 0
    Enable = 1


class PV_FocusMode:
    Auto = 0
    Single = 1
    Continuous = 2
    Manual = 3


class PV_AutoFocusRange:
    FullRange = 0
    Macro = 1
    Normal = 2


class PV_ManualFocusDistance:
    Infinity = 0
    Nearest = 2


class PV_FocusValue:
    Min = 170
    Max = 10000


class PV_DriverFallback:
    Enable = 0
    Disable = 1


class PV_VideoTemporalDenoisingMode:
    Off = 0
    On = 1


class PV_ColorTemperaturePreset:
    Auto = 0
    Manual = 1
    Cloudy = 2
    Daylight = 3
    Flash = 4
    Fluorescent = 5
    Tungsten = 6
    Candlelight = 7


class PV_WhiteBalanceValue:
    Min = 2300 // 25
    Max = 7500 // 25


class PV_ExposureMode:
    Manual = 0
    Auto = 1
    

class PV_ExposureValue:
    Min = 1000 // 10
    Max = 660000 // 10


class PV_ExposurePriorityVideo:
    Disabled = 0
    Enabled = 1


class PV_IsoSpeedMode:
    Manual = 0
    Auto = 1


class PV_IsoSpeedValue:
    Min = 100
    Max = 3200


class PV_BacklightCompensationState:
    Disable = 0
    Enable = 1


class PV_CaptureSceneMode:
    Auto = 0
    Macro = 2
    Portrait = 3
    Sport = 4
    Snow = 5
    Night = 6
    Beach = 7
    Sunset = 8
    Candlelight = 9
    Landscape = 10
    NightPortrait = 11
    Backlit = 12


class PV_MediaCaptureOptimization:
    Default = 0
    Quality = 1
    Latency = 2
    Power = 3
    LatencyThenQuality = 4
    LatencyThenPower = 5
    PowerAndQuality = 6


class PV_CaptureUse:
    NotSet = 0
    Photo = 1
    Video = 2


class PV_OpticalImageStabilizationMode:
    Off = 0
    On = 1


class PV_HdrVideoMode:
    Off = 0
    On = 1
    Auto = 2


class PV_RegionOfInterestWeight:
    Min = 0
    Max = 100


class PV_RegionOfInterestType:
    Unknown = 0
    Face = 1


class EE_InterfacePriority:
    LOWEST = -2
    BELOW_NORMAL = -1
    NORMAL = 0
    ABOVE_NORMAL = 1
    HIGHEST = 2


class RM_MapCameraPointOperation:
    ImagePointToCameraUnitPlane = 0
    CameraSpaceToImagePoint = 1


class TS_Source:
    QPC = 0
    UTC = 1


class ipc_rc(_context_manager):
    _CMD_EE_GET_APPLICATION_VERSION = 0x00
    _CMD_TS_GET_UTC_OFFSET = 0x01
    _CMD_HS_SET_MARKER_STATE = 0x02
    _CMD_PV_GET_SUBSYSTEM_STATUS = 0x03
    _CMD_PV_SET_FOCUS = 0x04
    _CMD_PV_SET_VIDEO_TEMPORAL_DENOISING = 0x05
    _CMD_PV_SET_WHITE_BALANCE_PRESET = 0x06
    _CMD_PV_SET_WHITE_BALANCE_VALUE = 0x07
    _CMD_PV_SET_EXPOSURE = 0x08
    _CMD_PV_SET_EXPOSURE_PRIORITY_VIDEO = 0x09
    _CMD_PV_SET_ISO_SPEED = 0x0A
    _CMD_PV_SET_BACKLIGHT_COMPENSATION = 0x0B
    _CMD_PV_SET_SCENE_MODE = 0x0C
    _CMD_EE_SET_FLAT_MODE = 0x0D
    _CMD_RM_SET_EYE_SELECTION = 0x0E
    _CMD_PV_SET_DESIRED_OPTIMIZATION = 0x0F
    _CMD_PV_SET_PRIMARY_USE = 0x10
    _CMD_PV_SET_OPTICAL_IMAGE_STABILIZATION = 0x11
    _CMD_PV_SET_HDR_VIDEO = 0x12
    _CMD_PV_SET_REGIONS_OF_INTEREST = 0x13
    _CMD_EE_SET_INTERFACE_PRIORITY = 0x14
    _CMD_EE_SET_QUIET_MODE = 0x15
    _CMD_RM_MAP_CAMERA_POINTS = 0x16
    _CMD_RM_GET_RIGNODE_WORLD_POSES = 0x17
    _CMD_TS_GET_CURRENT_TIME = 0x18
    _CMD_SI_SET_SAMPLING_DELAY = 0x19

    def __init__(self, host, port, sockopt):
        self.host = host
        self.port = port
        self.sockopt = sockopt

    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port, self.sockopt)

    def close(self):
        self._client.close()

    def ee_get_application_version(self):
        command = struct.pack('<B', ipc_rc._CMD_EE_GET_APPLICATION_VERSION)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.SHORT * 4, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<HHHH', data)

    def ts_get_utc_offset(self):
        command = struct.pack('<B', ipc_rc._CMD_TS_GET_UTC_OFFSET)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.LONGLONG, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<Q', data)[0]

    def hs_set_marker_state(self, state):
        command = struct.pack('<BI', ipc_rc._CMD_HS_SET_MARKER_STATE, state)
        self._client.sendall(command)

    def pv_get_subsystem_status(self):
        command = struct.pack('<B', ipc_rc._CMD_PV_GET_SUBSYSTEM_STATUS)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.BYTE, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<B', data)[0] != 0
    
    def pv_wait_for_subsystem(self, status):
        while (self.pv_get_subsystem_status() != status):
            pass

    def pv_set_focus(self, focusmode, autofocusrange, distance, value, driverfallback):
        command = struct.pack('<BIIIII', ipc_rc._CMD_PV_SET_FOCUS, focusmode, autofocusrange, distance, value, driverfallback)
        self._client.sendall(command)

    def pv_set_video_temporal_denoising(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_VIDEO_TEMPORAL_DENOISING, mode)
        self._client.sendall(command)

    def pv_set_white_balance_preset(self, preset):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_WHITE_BALANCE_PRESET, preset)
        self._client.sendall(command)

    def pv_set_white_balance_value(self, value):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_WHITE_BALANCE_VALUE, value)
        self._client.sendall(command)

    def pv_set_exposure(self, mode, value):
        command = struct.pack('<BII', ipc_rc._CMD_PV_SET_EXPOSURE, mode, value)
        self._client.sendall(command)
    
    def pv_set_exposure_priority_video(self, enabled):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_EXPOSURE_PRIORITY_VIDEO, enabled)
        self._client.sendall(command)

    def pv_set_iso_speed(self, mode, value):
        command = struct.pack('<BII', ipc_rc._CMD_PV_SET_ISO_SPEED, mode, value)
        self._client.sendall(command)

    def pv_set_backlight_compensation(self, state):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_BACKLIGHT_COMPENSATION, state)
        self._client.sendall(command)

    def pv_set_scene_mode(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_SCENE_MODE, mode)
        self._client.sendall(command)

    def ee_set_flat_mode(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_EE_SET_FLAT_MODE, mode)
        self._client.sendall(command)

    def rm_set_eye_selection(self, enable):
        command = struct.pack('<BI', ipc_rc._CMD_RM_SET_EYE_SELECTION, 1 if (enable) else 0)
        self._client.sendall(command)

    def pv_set_desired_optimization(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_DESIRED_OPTIMIZATION, mode)
        self._client.sendall(command)

    def pv_set_primary_use(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_PRIMARY_USE, mode)
        self._client.sendall(command)

    def pv_set_optical_image_stabilization(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_OPTICAL_IMAGE_STABILIZATION, mode)
        self._client.sendall(command)

    def pv_set_hdr_video(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_PV_SET_HDR_VIDEO, mode)
        self._client.sendall(command)

    def pv_set_regions_of_interest(self, clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h):
        mode = (0x1000 if (clear) else 0) | (0x0800 if (set) else 0) | (0x0400 if (auto_exposure) else 0) | (0x0200 if (auto_focus) else 0) | (0x0100 if (bounds_normalized) else 0) | ((type & 1) << 7) | (weight & 0x007F)
        command = struct.pack('<BIffff', ipc_rc._CMD_PV_SET_REGIONS_OF_INTEREST, mode, x, y, w, h)
        self._client.sendall(command)

    def ee_set_interface_priority(self, port, priority):
        command = struct.pack('<BIi', ipc_rc._CMD_EE_SET_INTERFACE_PRIORITY, port, priority)
        self._client.sendall(command)

    def ee_set_quiet_mode(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_EE_SET_QUIET_MODE, mode)
        self._client.sendall(command)

    def rm_map_camera_points(self, port, operation, points):
        data = points.tobytes()
        count = len(data) // (2 * _SIZEOF.FLOAT)
        size = count * 2 * _SIZEOF.FLOAT
        command = struct.pack('<BIII', ipc_rc._CMD_RM_MAP_CAMERA_POINTS, port, operation, count) + data[:size]
        self._client.sendall(command)
        response = self._client.download(size, ChunkSize.SINGLE_TRANSFER)
        return np.frombuffer(response, dtype=np.float32).reshape(points.shape)

    def rm_get_rignode_world_poses(self, timestamps):
        data = timestamps.tobytes()
        count = len(data) // _SIZEOF.QWORD
        size_in = count * _SIZEOF.QWORD
        size_out = count * 4 * 4 * _SIZEOF.FLOAT
        command = struct.pack('<BI', ipc_rc._CMD_RM_GET_RIGNODE_WORLD_POSES, count) + data[:size_in]
        self._client.sendall(command)
        response = self._client.download(size_out, ChunkSize.SINGLE_TRANSFER)
        return np.frombuffer(response, dtype=np.float32).reshape((count, 4, 4))

    def ts_get_current_time(self, source):
        command = struct.pack('<BI', ipc_rc._CMD_TS_GET_CURRENT_TIME, source)
        self._client.sendall(command)
        response = self._client.download(_SIZEOF.QWORD, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<Q', response)[0]
    
    def si_set_sampling_delay(self, delay):
        command = struct.pack('<Bq', ipc_rc._CMD_SI_SET_SAMPLING_DELAY, delay)
        self._client.sendall(command)


#------------------------------------------------------------------------------
# Spatial Mapping
#------------------------------------------------------------------------------

class _SM_VolumeType:
    Box         = 0
    Frustum     = 1
    OrientedBox = 2
    Sphere      = 3


class SM_VertexPositionFormat:
    R32G32B32A32Float         = 2
    R16G16B16A16IntNormalized = 13


class SM_TriangleIndexFormat:
    R16UInt = 57
    R32Uint = 42


class SM_VertexNormalFormat:
    R32G32B32A32Float     = 2
    R8G8B8A8IntNormalized = 31


class _SM_Convert:
    DirectXPixelFormatToNumPy = { 2 : np.float32, 13 : np.int16, 57 : np.uint16, 42 : np.uint32, 31 : np.int8 }


class sm_bounding_volume:
    def __init__(self):
        self._count = 0
        self._data = bytearray()

    def _add(self, data):
        self._data.extend(data)
        self._count += 1

    def add_box(self, center, extents):
        self._add(struct.pack('<Iffffff', _SM_VolumeType.Box, center[0], center[1], center[2], extents[0], extents[1], extents[2]))

    def add_frustum(self, near, far, right, left, top, bottom):
        self._add(struct.pack('<Iffffffffffffffffffffffff', _SM_VolumeType.Frustum, near[0], near[1], near[2], near[3], far[0], far[1], far[2], far[3], right[0], right[1], right[2], right[3], left[0], left[1], left[2], left[3], top[0], top[1], top[2], top[3], bottom[0], bottom[1], bottom[2], bottom[3]))

    def add_oriented_box(self, center, extents, orientation):
        self._add(struct.pack('<Iffffffffff', _SM_VolumeType.OrientedBox, center[0], center[1], center[2], extents[0], extents[1], extents[2], orientation[0], orientation[1], orientation[2], orientation[3]))

    def add_sphere(self, center, radius):
        self._add(struct.pack('<Iffff', _SM_VolumeType.Sphere, center[0], center[1], center[2], radius))

    def _get(self):
        return self._count, self._data


class _sm_surface_info:
    def __init__(self, id, update_time):
        self.id = id
        self.update_time = update_time


class sm_mesh_task:
    def __init__(self):
        self._count = 0
        self._data = bytearray()
        self._vpf = []
        self._tif = []
        self._vnf = []

    def add_task(self, id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format):
        self._data.extend(struct.pack('<16sdIIII', id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format, 0))
        self._vpf.append(vertex_position_format)
        self._tif.append(triangle_index_format)
        self._vnf.append(vertex_normal_format)
        self._count += 1

    def _get(self):
        return self._count, self._data, self._vpf, self._tif, self._vnf


class _sm_mesh:
    def __init__(self, vertex_position_scale, pose, bounds, vertex_positions, triangle_indices, vertex_normals):
        self.vertex_position_scale = vertex_position_scale
        self.pose                  = pose
        self.bounds                = bounds        
        self.vertex_positions      = vertex_positions
        self.triangle_indices      = triangle_indices
        self.vertex_normals        = vertex_normals


def _sm_mesh_unpack(vertex_position_format, triangle_index_format, vertex_normal_format, vertex_position_scale, pose, bounds, vertex_positions, triangle_indices, vertex_normals):
    vertex_position_scale = np.frombuffer(vertex_position_scale, dtype=np.float32).reshape((1, 3))
    pose                  = np.frombuffer(pose,                  dtype=np.float32).reshape((4, 4))
    bounds                = np.frombuffer(bounds,                dtype=np.float32)        
    vertex_positions      = np.frombuffer(vertex_positions,      dtype=_SM_Convert.DirectXPixelFormatToNumPy[vertex_position_format]).reshape((-1, 4))
    triangle_indices      = np.frombuffer(triangle_indices,      dtype=_SM_Convert.DirectXPixelFormatToNumPy[triangle_index_format]).reshape((-1, 3))
    vertex_normals        = np.frombuffer(vertex_normals,        dtype=_SM_Convert.DirectXPixelFormatToNumPy[vertex_normal_format]).reshape((-1, 4))
 
    return _sm_mesh(vertex_position_scale, pose, bounds, vertex_positions, triangle_indices, vertex_normals)


class ipc_sm(_context_manager):
    _CMD_SET_VOLUMES           = 0x00
    _CMD_GET_OBSERVED_SURFACES = 0x01
    _CMD_GET_MESHES            = 0x02

    def __init__(self, host, port, sockopt):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        
    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port, self.sockopt)

    def set_volumes(self, volumes):
        count, data = volumes._get()
        msg = bytearray()
        msg.extend(struct.pack('<BB', ipc_sm._CMD_SET_VOLUMES, count))
        msg.extend(data)
        self._client.sendall(msg)

    def get_observed_surfaces(self):
        self._client.sendall(struct.pack('<B', ipc_sm._CMD_GET_OBSERVED_SURFACES))
        count = struct.unpack('<I', self._client.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))[0]
        ids = self._client.download(count * 24, ChunkSize.SINGLE_TRANSFER)
        return [_sm_surface_info(ids[(i*24):(i*24+16)], struct.unpack('<Q', ids[(i*24+16):(i*24+24)])[0]) for i in range(0, count)]
    
    def _download_mesh(self, vpf, tif, vnf):
        header = self._client.download(136, ChunkSize.SINGLE_TRANSFER)

        index, status, vpl, til, vnl = struct.unpack('<IIIII', header[:20])
        scale = header[20:32]
        pose = header[32:96]
        bounds = header[96:(96+40)]

        if (status != 0):
            return index, None
        
        payload = self._client.download(vpl + til + vnl, ChunkSize.SINGLE_TRANSFER)

        vpd_b = 0
        vpd_e = vpd_b + vpl
        tid_b = vpd_e
        tid_e = tid_b + til
        vnd_b = tid_e
        vnd_e = vnd_b + vnl

        vertex_positions = payload[vpd_b:vpd_e]
        triangle_indices = payload[tid_b:tid_e]
        vertex_normals   = payload[vnd_b:vnd_e]

        return index, _sm_mesh_unpack(vpf[index], tif[index], vnf[index], scale, pose, bounds, vertex_positions, triangle_indices, vertex_normals)
    
    def _download_meshes(self, count, vpf, tif, vnf):
        for _ in range(0, count):
            yield self._download_mesh(vpf, tif, vnf)
    
    def get_meshes(self, tasks):
        count, data, vpf, tif, vnf = tasks._get()
        msg = bytearray()
        msg.extend(struct.pack('<BI', ipc_sm._CMD_GET_MESHES, count))
        msg.extend(data)
        self._client.sendall(msg)
        meshes = {index : mesh for index, mesh in self._download_meshes(count, vpf, tif, vnf)}
        return meshes

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Scene Understanding
#------------------------------------------------------------------------------

class SU_MeshLOD:
    Coarse = 0
    Medium = 1
    Fine = 2
    Unlimited = 255


class SU_KindFlag:
    Background = 1
    Wall = 2
    Floor = 4
    Ceiling = 8
    Platform = 16
    Unknown = 32
    World = 64
    CompletelyInferred = 128


class SU_Create:
    New = 0
    NewFromPrevious = 1


class SU_Kind:
    Background = 0
    Wall = 1
    Floor = 2
    Ceiling = 3
    Platform = 4
    Unknown = 247
    World = 248
    CompletelyInferred = 249


class su_task:
    def __init__(self, enable_quads, enable_meshes, enable_only_observed, enable_world_mesh, mesh_lod, query_radius, create_mode, kind_flags, get_orientation, get_position, get_location_matrix, get_quad, get_meshes, get_collider_meshes, guid_list):
        self.enable_quads = enable_quads
        self.enable_meshes = enable_meshes
        self.enable_only_observed = enable_only_observed
        self.enable_world_mesh = enable_world_mesh
        self.mesh_lod = mesh_lod
        self.query_radius = query_radius
        self.create_mode = create_mode
        self.kind_flags = kind_flags
        self.get_orientation = get_orientation
        self.get_position = get_position
        self.get_location_matrix = get_location_matrix
        self.get_quad = get_quad
        self.get_meshes = get_meshes
        self.get_collider_meshes = get_collider_meshes
        self.guid_list = guid_list

    def _get(self):
        task = bytearray()
        task.extend(struct.pack('<BBBBIfBBBBBBBBI', self.enable_quads, self.enable_meshes, self.enable_only_observed, self.enable_world_mesh, self.mesh_lod, self.query_radius, self.create_mode, self.kind_flags, self.get_orientation, self.get_position, self.get_location_matrix, self.get_quad, self.get_meshes, self.get_collider_meshes, len(self.guid_list)))
        for guid in self.guid_list:
            task.extend(guid)
        return task


class _su_mesh:
    def __init__(self, vertex_positions, triangle_indices):
        self.vertex_positions = vertex_positions
        self.triangle_indices = triangle_indices


def _su_mesh_unpack(vertex_positions, triangle_indices):
    vertex_positions = np.frombuffer(vertex_positions, dtype=np.float32).reshape((-1, 3))
    triangle_indices = np.frombuffer(triangle_indices, dtype=np.uint32).reshape((-1, 3))

    return _su_mesh(vertex_positions, triangle_indices)


class _su_item:
    def __init__(self, id, kind, orientation, position, location, alignment, extents, meshes, collider_meshes):
        self.id = id
        self.kind = kind
        self.orientation = orientation
        self.position = position
        self.location = location
        self.alignment = alignment
        self.extents = extents
        self.meshes = meshes
        self.collider_meshes = collider_meshes


def _su_item_unpack(id, kind, orientation, position, location, alignment, extents, meshes, collider_meshes):
    kind = np.frombuffer(kind, dtype=np.int32)
    orientation = np.frombuffer(orientation, dtype=np.float32)
    position = np.frombuffer(position, dtype=np.float32)
    location = np.frombuffer(location, dtype=np.float32).reshape((-1, 4))
    alignment = np.frombuffer(alignment, dtype=np.int32)
    extents = np.frombuffer(extents, dtype=np.float32)

    return _su_item(id, kind, orientation, position, location, alignment, extents, meshes, collider_meshes)


class _su_result:
    def __init__(self, extrinsics, pose, items):        
        self.extrinsics = extrinsics
        self.pose = pose
        self.items = items


def _su_result_unpack(extrinsics, pose, items):
    extrinsics = np.frombuffer(extrinsics, dtype=np.float32).reshape((4, 4))
    pose = np.frombuffer(pose, dtype=np.float32).reshape((4, 4))

    return _su_result(extrinsics, pose, items)


class ipc_su(_context_manager):
    def __init__(self, host, port, sockopt):
        self.host = host
        self.port = port
        self.sockopt = sockopt
        
    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port, self.sockopt)

    def _download_mesh(self):
        elements_vertices, elements_indices = struct.unpack('<II', self._client.download(2 * _SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))
        vpl = elements_vertices * _SIZEOF.DWORD
        til = elements_indices * _SIZEOF.DWORD
        data = self._client.download(vpl + til, ChunkSize.SINGLE_TRANSFER)
        return _su_mesh_unpack(data[:vpl], data[vpl:])

    def _download_meshes(self):
        return [self._download_mesh() for _ in range(0, struct.unpack('<I', self._client.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))[0])]
    
    def _download_item(self, bi, bk, bo, bp, bl, ba, be, bm, download_meshes, download_collider_meshes):
        d = self._client.download(bm, ChunkSize.SINGLE_TRANSFER)
        return _su_item_unpack(d[bi:bk], d[bk:bo], d[bo:bp], d[bp:bl], d[bl:ba], d[ba:be], d[be:bm], self._download_meshes() if (download_meshes) else [], self._download_meshes() if (download_collider_meshes) else [])
    
    def query(self, task):
        self._client.sendall(task._get())
        header = self._client.download(136, ChunkSize.SINGLE_TRANSFER)
        status = struct.unpack('<I', header[:4])[0]
        if (status != 0):
            return None
        he = 4
        hp = he + 64
        hi = hp + 64
        bi = 0
        bk = bi + 16
        bo = bk + 4
        bp = bo + (16 * task.get_orientation)
        bl = bp + (12 * task.get_position)
        ba = bl + (64 * task.get_location_matrix)
        be = ba + (4 * task.get_quad)
        bm = be + (8 * task.get_quad)
        return _su_result_unpack(header[he:hp], header[hp:hi], [self._download_item(bi, bk, bo, bp, bl, ba, be, bm, task.get_meshes, task.get_collider_meshes) for _ in range(0, struct.unpack('<I', header[132:])[0])])

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Voice Input
#------------------------------------------------------------------------------

class VI_SpeechRecognitionConfidence:
    High = 0
    Medium = 1
    Low = 2
    Rejected = 3


class _vi_result:
    def __init__(self, index, confidence, phrase_duration, phrase_start_time, raw_confidence):
        self.index = index
        self.confidence = confidence
        self.phrase_duration = phrase_duration
        self.phrase_start_time = phrase_start_time
        self.raw_confidence = raw_confidence


def _vi_result_unpack(index, confidence, phrase_duration, phrase_start_time, raw_confidence):
    index = struct.unpack('<I', index)[0]
    confidence = struct.unpack('<I', confidence)[0]
    phrase_duration = struct.unpack('<Q', phrase_duration)[0]
    phrase_start_time = struct.unpack('<Q', phrase_start_time)[0]
    raw_confidence = struct.unpack('<d', raw_confidence)[0]

    return _vi_result(index, confidence, phrase_duration, phrase_start_time, raw_confidence)


class ipc_vi(_context_manager):
    _CMD_POP  = 0x01
    _CMD_STOP = 0x00

    def __init__(self, host, port, sockopt):
        self.host = host
        self.port = port
        self.sockopt = sockopt

    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port, self.sockopt)

    def start(self, strings):
        command = bytearray()
        command.extend(struct.pack('<H', len(strings)))
        for string in strings:
            encoded = string.encode('utf-16')
            command.extend(struct.pack('<H', len(encoded)))
            command.extend(encoded)
        self._client.sendall(command)
        self._strings = strings

    def pop(self):
        command = struct.pack('<B', ipc_vi._CMD_POP)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER)
        count = struct.unpack('<I', data)[0]
        data = self._client.download(32*count, ChunkSize.SINGLE_TRANSFER)
        return [_vi_result_unpack(data[(i*32):(i*32+4)], data[(i*32+4):(i*32+8)], data[(i*32+8):(i*32+16)], data[(i*32+16):(i*32+24)], data[(i*32+24):(i*32+32)]) for i in range(0, count)]

    def stop(self):
        command = struct.pack('<B', ipc_vi._CMD_STOP)
        self._client.sendall(command)

    def translate(self, index):
        if ((index >= 0) and (index < len(self._strings))):
            return self._strings[index]
        return None

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Unity Message Queue
#------------------------------------------------------------------------------

class umq_command_buffer:
    def __init__(self):
        self._buffer = bytearray()
        self._count = 0

    def add(self, id, data):
        self._buffer.extend(struct.pack('<II', id, len(data)))
        self._buffer.extend(data)
        self._count += 1

    def get_data(self):
        return bytes(self._buffer)
    
    def get_count(self):
        return self._count


class ipc_umq(_context_manager):
    def __init__(self, host, port, sockopt):
        self.host = host
        self.port = port
        self.sockopt = sockopt
    
    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port, self.sockopt)

    def push(self, buffer):
        self._client.sendall(buffer.get_data())
    
    def pull(self, buffer):
        return np.frombuffer(self._client.download(_SIZEOF.DWORD * buffer.get_count(), ChunkSize.SINGLE_TRANSFER), dtype=np.uint32)

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Guest Message Queue
#------------------------------------------------------------------------------

class _gmq_message:
    def __init__(self, id, data):
        self.id   = id
        self.data = data


class ipc_gmq(_context_manager):
    _CMD_NONE = _RANGEOF.U32_MAX

    def __init__(self, host, port, sockopt):
        self.host = host
        self.port = port
        self.sockopt = sockopt

    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port, self.sockopt)

    def pull(self):
        self._client.sendall(struct.pack('<I', ipc_gmq._CMD_NONE))
        header = struct.unpack('<II', self._client.download(_SIZEOF.DWORD * 2, ChunkSize.SINGLE_TRANSFER))
        data = self._client.download(header[1], ChunkSize.SINGLE_TRANSFER) if (header[1] > 0) else b''
        return _gmq_message(header[0], data) if (header[0] != ipc_gmq._CMD_NONE) else None
    
    def push(self, response):
        self._client.sendall(struct.pack('<I', response))
    
    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Timestamps
#------------------------------------------------------------------------------

def ts_qpc_to_filetime(timestamp, utc_offset):
    return timestamp + utc_offset


def ts_filetime_to_unix_hns(timestamp_filetime):
    return timestamp_filetime - TimeBase.UNIX_EPOCH


def ts_unix_hns_to_unix(timestamp_unix_hns):
    return timestamp_unix_hns / TimeBase.HUNDREDS_OF_NANOSECONDS


def ts_unix_to_unix_hns(timestamp_unix):
    return int(timestamp_unix * TimeBase.HUNDREDS_OF_NANOSECONDS)


def ts_unix_hns_to_filetime(timestamp_unix_hns):
    return timestamp_unix_hns + TimeBase.UNIX_EPOCH


def ts_filetime_to_qpc(timestamp_filetime, utc_offset):
    return timestamp_filetime - utc_offset

