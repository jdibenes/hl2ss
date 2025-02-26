
import numpy as np
import socket
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
    MICROPHONE           = 512
    SPATIAL_INPUT        = 1024
    EXTENDED_EYE_TRACKER = 256
    EXTENDED_AUDIO       = 512
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


class HologramPerspective:
    DISPLAY = 0
    PV      = 1


class MixerMode:
    MICROPHONE = 0
    SYSTEM     = 1
    BOTH       = 2
    QUERY      = 3


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
    ARRAY_CHANNELS     = 5
    ARRAY_TOP_LEFT     = 0
    ARRAY_TOP_CENTER   = 1
    ARRAY_TOP_RIGHT    = 2
    ARRAY_BOTTOM_LEFT  = 3
    ARRAY_BOTTOM_RIGHT = 4

    SAMPLE_RATE    = 48000
    CHANNELS       = 2
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
    def open(self, host, port):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((host, port))

    def sendall(self, data):
        self._socket.sendall(data)

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
    if (packet.pose is not None):
        buffer.extend(packet.pose.tobytes())
    return buffer


def unpack_packet(data):
    timestamp, payload_size = struct.unpack('<QI', data[:12])
    payload = data[12:(12 + payload_size)]
    pose = data[(12 + payload_size):]
    return _packet(timestamp, payload, np.frombuffer(pose, dtype=np.float32).reshape((4, 4)) if (len(pose) == 64) else None)


def is_valid_pose(pose):
    return pose[3, 3] != 0


class _unpacker:
    def reset(self, mode):
        self._mode = mode
        self._state = 0
        self._buffer = bytearray()
        self._timestamp = None
        self._size = None
        self._payload = None
        self._pose = None

    def extend(self, chunk):
        self._buffer.extend(chunk)

    def unpack(self):        
        length = len(self._buffer)
        
        while (True):
            if (self._state == 0):
                if (length >= 12):
                    header = struct.unpack('<QI', self._buffer[:12])
                    self._timestamp = header[0]
                    self._size = 12 + header[1]
                    if (self._mode == StreamMode.MODE_1):
                        self._size += 64
                    self._state = 1
                    continue
            elif (self._state == 1):
                if (length >= self._size):
                    if (self._mode == StreamMode.MODE_1):
                        payload_end = self._size - 64
                        self._pose = np.frombuffer(self._buffer[payload_end:self._size], dtype=np.float32).reshape((4, 4))
                    else:
                        payload_end = self._size
                    self._payload = self._buffer[12:payload_end]
                    self._buffer = self._buffer[self._size:]
                    self._state = 0
                    return True
            return False

    def get(self):
        return _packet(self._timestamp, self._payload, self._pose)


#------------------------------------------------------------------------------
# Packet Gatherer
#------------------------------------------------------------------------------

class _gatherer:
    def open(self, host, port, chunk_size, mode):
        self._client = _client()
        self._unpacker = _unpacker()
        self._chunk_size = chunk_size
        self._unpacker.reset(mode)
        self._client.open(host, port)
        
    def sendall(self, data):
        self._client.sendall(data)

    def get_next_packet(self):
        while (True):
            self._unpacker.extend(self._client.recv(self._chunk_size))
            if (self._unpacker.unpack()):
                return self._unpacker.get()

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Stream Configuration
#------------------------------------------------------------------------------

class H26xEncoderProperty:
    CODECAPI_AVEncCommonRateControlMode = 0
    CODECAPI_AVEncCommonQuality = 1
    CODECAPI_AVEncAdaptiveMode = 2
    CODECAPI_AVEncCommonBufferSize = 3
    CODECAPI_AVEncCommonMaxBitRate = 4
    CODECAPI_AVEncCommonMeanBitRate = 5
    CODECAPI_AVEncCommonQualityVsSpeed = 6
    CODECAPI_AVEncH264CABACEnable = 7
    CODECAPI_AVEncH264SPSID = 8
    CODECAPI_AVEncMPVDefaultBPictureCount = 9
    CODECAPI_AVEncMPVGOPSize = 10
    CODECAPI_AVEncNumWorkerThreads = 11 
    CODECAPI_AVEncVideoContentType = 12
    CODECAPI_AVEncVideoEncodeQP = 13
    CODECAPI_AVEncVideoForceKeyFrame = 14 
    CODECAPI_AVEncVideoMinQP = 15
    CODECAPI_AVLowLatencyMode = 16
    CODECAPI_AVEncVideoMaxQP = 17
    CODECAPI_VideoEncoderDisplayContentType = 18
    HL2SSAPI_VideoMediaIndex            = 0xFFFFFFFFFFFFFFFB
    HL2SSAPI_VideoStrideMask            = 0xFFFFFFFFFFFFFFFC
    HL2SSAPI_AcquisitionMode            = 0xFFFFFFFFFFFFFFFD
    HL2SSAPI_VLCHostTicksOffsetConstant = 0xFFFFFFFFFFFFFFFE
    HL2SSAPI_VLCHostTicksOffsetExposure = 0xFFFFFFFFFFFFFFFF


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


def extended_audio_device_mixer_mode(mixer_mode, device):
    DEVICE_BASE = 0x00000004
    return mixer_mode | (DEVICE_BASE * (device + 1))


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Acquisition
#------------------------------------------------------------------------------

def _connect_client_rm_vlc(host, port, chunk_size, mode, divisor, profile, level, bitrate, options):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_vlc(mode, divisor, profile, level, bitrate, options))
    return c


def _connect_client_rm_depth_ahat(host, port, chunk_size, mode, divisor, profile_z, profile_ab, level, bitrate, options):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_depth_ahat(mode, divisor, profile_z, profile_ab, level, bitrate, options))
    return c


def _connect_client_rm_depth_longthrow(host, port, chunk_size, mode, divisor, png_filter):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_depth_longthrow(mode, divisor, png_filter))
    return c


def _connect_client_rm_imu(host, port, chunk_size, mode):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_imu(mode))
    return c


def _connect_client_pv(host, port, chunk_size, mode, width, height, framerate, divisor, profile, level, bitrate, options):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_pv(mode, width, height, framerate, divisor, profile, level, bitrate, options))
    return c


def _connect_client_microphone(host, port, chunk_size, profile, level):
    c = _gatherer()
    c.open(host, port, chunk_size, StreamMode.MODE_0)
    c.sendall(_create_configuration_for_microphone(profile, level))
    return c


def _connect_client_si(host, port, chunk_size):
    c = _gatherer()
    c.open(host, port, chunk_size, StreamMode.MODE_0)
    return c


def _connect_client_eet(host, port, chunk_size, fps):
    c = _gatherer()
    c.open(host, port, chunk_size, StreamMode.MODE_1)
    c.sendall(_create_configuration_for_eet(fps))
    return c


def _connect_client_extended_audio(host, port, chunk_size, mixer_mode, loopback_gain, microphone_gain, profile, level):
    c = _gatherer()
    c.open(host, port, chunk_size, StreamMode.MODE_0)
    c.sendall(_create_configuration_for_extended_audio(mixer_mode, loopback_gain, microphone_gain, profile, level))
    return c


def _connect_client_extended_depth(host, port, chunk_size, mode, divisor, profile_z, options):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_extended_depth(mode, divisor, profile_z, options))
    return c
  

class _PVCNT:
    START =  0x04
    STOP   = 0x08


def start_subsystem_pv(host, port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective):
    c = _client()
    c.open(host, port)
    c.sendall(_create_configuration_for_mode(_PVCNT.START | StreamMode.MODE_3))
    c.sendall(_create_configuration_for_mrc_video(enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective))
    c.close()


def stop_subsystem_pv(host, port):
    c = _client()
    c.open(host, port)
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
    def __init__(self, host, port, chunk, mode, divisor, profile, level, bitrate, options):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile = profile
        self.level = level
        self.bitrate = bitrate
        self.options = options

    def open(self):
        self._client = _connect_client_rm_vlc(self.host, self.port, self.chunk, self.mode, self.divisor, self.profile, self.level, self.bitrate, self.options)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_rm_depth_ahat(_context_manager):
    def __init__(self, host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile_z = profile_z
        self.profile_ab = profile_ab
        self.level = level
        self.bitrate = bitrate
        self.options = options

    def open(self):
        self._client = _connect_client_rm_depth_ahat(self.host, self.port, self.chunk, self.mode, self.divisor, self.profile_z, self.profile_ab, self.level, self.bitrate, self.options)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_rm_depth_longthrow(_context_manager):
    def __init__(self, host, port, chunk, mode, divisor, png_filter):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.png_filter = png_filter

    def open(self):
        self._client = _connect_client_rm_depth_longthrow(self.host, self.port, self.chunk, self.mode, self.divisor, self.png_filter)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_rm_imu(_context_manager):
    def __init__(self, host, port, chunk, mode):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode

    def open(self):
        self._client = _connect_client_rm_imu(self.host, self.port, self.chunk, self.mode)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_pv(_context_manager):
    def __init__(self, host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options):
        self.host = host
        self.port = port
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
        self._client = _connect_client_pv(self.host, self.port, self.chunk, self.mode, self.width, self.height, self.framerate, self.divisor, self.profile, self.level, self.bitrate, self.options)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_microphone(_context_manager):
    def __init__(self, host, port, chunk, profile, level):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.profile = profile
        self.level = level

    def open(self):
        self._client = _connect_client_microphone(self.host, self.port, self.chunk, self.profile, self.level)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_si(_context_manager):
    def __init__(self, host, port, chunk):
        self.host = host
        self.port = port
        self.chunk = chunk

    def open(self):
        self._client = _connect_client_si(self.host, self.port, self.chunk)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_eet(_context_manager):
    def __init__(self, host, port, chunk, fps):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.fps = fps

    def open(self):
        self._client = _connect_client_eet(self.host, self.port, self.chunk, self.fps)

    def get_next_packet(self):
        return self._client.get_next_packet()
    
    def close(self):
        self._client.close()


class rx_extended_audio:
    def __init__(self, host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mixer_mode = mixer_mode
        self.loopback_gain = loopback_gain
        self.microphone_gain = microphone_gain
        self.profile = profile
        self.level = level

    def open(self):
        self._client = _connect_client_extended_audio(self.host, self.port, self.chunk, self.mixer_mode, self.loopback_gain, self.microphone_gain, self.profile, self.level)

    def get_next_packet(self):
        return self._client.get_next_packet()
    
    def close(self):
        self._client.close()


class rx_extended_depth:
    def __init__(self, host, port, chunk, mode, divisor, profile_z, options):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile_z = profile_z
        self.options = options

    def open(self):
        self._client = _connect_client_extended_depth(self.host, self.port, self.chunk, self.mode, self.divisor, self.profile_z, self.options)

    def get_next_packet(self):
        return self._client.get_next_packet()

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


#------------------------------------------------------------------------------
# RM VLC Decoder
#------------------------------------------------------------------------------

class _RM_VLC_Frame:
    def __init__(self, image, sensor_ticks, exposure, gain):
        self.image        = image
        self.sensor_ticks = sensor_ticks
        self.exposure     = exposure
        self.gain         = gain


def unpack_rm_vlc(payload):
    image    = payload[:-24]
    metadata = payload[-24:]

    sensor_ticks = np.frombuffer(metadata, dtype=np.uint64, offset=0,  count=1)
    exposure     = np.frombuffer(metadata, dtype=np.uint64, offset=8,  count=1)
    gain         = np.frombuffer(metadata, dtype=np.uint32, offset=16, count=1)

    return _RM_VLC_Frame(image, sensor_ticks, exposure, gain)


class _decode_rm_vlc:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = get_video_codec(self.profile)

    def decode(self, payload):
        return self._codec.decode(payload).to_ndarray()[:Parameters_RM_VLC.HEIGHT, :Parameters_RM_VLC.WIDTH]


class _unpack_rm_vlc:
    def create(self):
        pass

    def decode(self, payload):
        return np.frombuffer(payload, dtype=np.uint8).reshape(Parameters_RM_VLC.SHAPE)
    

def decode_rm_vlc(profile):
    return _unpack_rm_vlc() if (profile == VideoProfile.RAW) else _decode_rm_vlc(profile)


#------------------------------------------------------------------------------
# RM Depth Decoder
#------------------------------------------------------------------------------

class _RM_Depth_Frame:
    def __init__(self, depth, ab, sensor_ticks):
        self.depth        = depth
        self.ab           = ab
        self.sensor_ticks = sensor_ticks


class _Mode0Layout_RM_DEPTH_AHAT_STRUCT:
    BASE = 8


class _Mode0Layout_RM_DEPTH_AHAT:
    BEGIN_DEPTH_Y = 0
    END_DEPTH_Y   = BEGIN_DEPTH_Y + Parameters_RM_DEPTH_AHAT.HEIGHT
    BEGIN_AB_U_Y  = END_DEPTH_Y
    END_AB_U_Y    = BEGIN_AB_U_Y + (Parameters_RM_DEPTH_AHAT.WIDTH // 4)
    BEGIN_AB_V_Y  = END_AB_U_Y
    END_AB_V_Y    = BEGIN_AB_V_Y + (Parameters_RM_DEPTH_AHAT.WIDTH // 4)


def _unpack_rm_depth_ahat_nv12_as_yuv420p(yuv, sensor_ticks):
    y = yuv[_Mode0Layout_RM_DEPTH_AHAT.BEGIN_DEPTH_Y : _Mode0Layout_RM_DEPTH_AHAT.END_DEPTH_Y, :]
    u = yuv[_Mode0Layout_RM_DEPTH_AHAT.BEGIN_AB_U_Y  : _Mode0Layout_RM_DEPTH_AHAT.END_AB_U_Y,  :].reshape((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH // 4))
    v = yuv[_Mode0Layout_RM_DEPTH_AHAT.BEGIN_AB_V_Y  : _Mode0Layout_RM_DEPTH_AHAT.END_AB_V_Y,  :].reshape((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH // 4))

    depth = np.multiply(y, 4, dtype=np.uint16)
    ab = np.empty((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH), dtype=np.uint16)

    u = np.square(u, dtype=np.uint16)
    v = np.square(v, dtype=np.uint16)

    ab[:, 0::4] = u
    ab[:, 1::4] = u
    ab[:, 2::4] = v
    ab[:, 3::4] = v

    return _RM_Depth_Frame(depth, ab, sensor_ticks)


class _decode_rm_depth_ahat:
    def __init__(self, profile):
        self.profile = profile
   
    def create(self):
        self._codec = get_video_codec(self.profile)

    def decode(self, payload):
        return _unpack_rm_depth_ahat_nv12_as_yuv420p(self._codec.decode(payload[_Mode0Layout_RM_DEPTH_AHAT_STRUCT.BASE:-8]).to_ndarray(), np.frombuffer(payload[-8:], dtype=np.uint64, offset=0, count=1))


class _unpack_rm_depth_ahat:
    def create(self):
        pass

    def decode(self, payload):
        depth        = np.frombuffer(payload,      dtype=np.uint16, offset=_Mode0Layout_RM_DEPTH_AHAT_STRUCT.BASE,                                                  count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
        ab           = np.frombuffer(payload,      dtype=np.uint16, offset=_Mode0Layout_RM_DEPTH_AHAT_STRUCT.BASE + Parameters_RM_DEPTH_AHAT.PIXELS * _SIZEOF.WORD, count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
        sensor_ticks = np.frombuffer(payload[-8:], dtype=np.uint64, offset=0,                                                                                       count=1)
        return _RM_Depth_Frame(depth, ab, sensor_ticks)


class _decompress_zdepth:
    def create(self):
        import pyzdepth
        self._codec = pyzdepth.DepthCompressor()

    def decode(self, payload):
        result, width, height, decompressed = self._codec.Decompress(bytes(payload))
        return np.frombuffer(decompressed, dtype=np.uint16).reshape((height, width))


class _decode_ab_rm_depth_ahat:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = get_video_codec(self.profile)

    def decode(self, payload):
        return np.square(self._codec.decode(payload).to_ndarray()[:Parameters_RM_DEPTH_AHAT.HEIGHT, :Parameters_RM_DEPTH_AHAT.WIDTH], dtype=np.uint16)


class _unpack_ab_rm_depth_ahat:
    def create(self):
        pass

    def decode(self, payload):
        return np.frombuffer(payload, dtype=np.uint16, offset=0, count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)


class _decode_rm_depth_ahat_zdepth:
    def __init__(self, profile):
        self._codec_z  = _decompress_zdepth()
        self._codec_ab = _unpack_ab_rm_depth_ahat() if (profile == VideoProfile.RAW) else _decode_ab_rm_depth_ahat(profile)

    def create(self):
        self._codec_z.create()
        self._codec_ab.create()

    def decode(self, payload):
        size_z, size_ab = struct.unpack_from('<II', payload, 0)

        start_z  = _Mode0Layout_RM_DEPTH_AHAT_STRUCT.BASE
        end_z    = start_z + size_z
        start_ab = end_z
        end_ab   = start_ab + size_ab

        depth        = self._codec_z.decode(payload[start_z:end_z])
        ab           = self._codec_ab.decode(payload[start_ab:end_ab])
        sensor_ticks = np.frombuffer(payload[-8:], dtype=np.uint64, offset=0, count=1)

        return _RM_Depth_Frame(depth, ab, sensor_ticks)


def decode_rm_depth_ahat(profile_z, profile_ab):
    return (_unpack_rm_depth_ahat() if (profile_ab == VideoProfile.RAW) else _decode_rm_depth_ahat(profile_ab)) if (profile_z == DepthProfile.SAME) else _decode_rm_depth_ahat_zdepth(profile_ab)


def decode_rm_depth_longthrow(payload):
    composite    = cv2.imdecode(np.frombuffer(payload[:-8], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    h, w, _      = composite.shape
    image        = composite.view(np.uint16).reshape((2*h, w))
    sensor_ticks = np.frombuffer(payload[-8:], dtype=np.uint64, offset=0, count=1)
    return _RM_Depth_Frame(image[:h, :], image[h:, :], sensor_ticks)


#------------------------------------------------------------------------------
# RM IMU Unpacker
#------------------------------------------------------------------------------

class _RM_IMU_Frame:
    def __init__(self, vinyl_hup_ticks, soc_ticks, x, y, z, temperature):
        self.vinyl_hup_ticks = vinyl_hup_ticks
        self.soc_ticks       = soc_ticks
        self.x               = x
        self.y               = y
        self.z               = z
        self.temperature     = temperature


class unpack_rm_imu:
    def __init__(self, payload):
        self._count = len(payload) // 32
        self._batch = payload

    def get_count(self):
        return self._count

    def get_frame(self, index):
        data = struct.unpack('<QQffff', self._batch[(index * 32):((index + 1) * 32)])
        return _RM_IMU_Frame(data[0], data[1], data[2], data[3], data[4], data[5])


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


def create_pv_intrinsics(focal_length, principal_point):
    return np.array([[-focal_length[0], 0, 0, 0], [0, focal_length[1], 0, 0], [principal_point[0], principal_point[1], 1, 0], [0, 0, 0, 1]], dtype=np.float32)


def create_pv_intrinsics_placeholder():
    return np.eye(4, 4, dtype=np.float32)


def update_pv_intrinsics(intrinsics, focal_length, principal_point):
    intrinsics[0, 0] = -focal_length[0]
    intrinsics[1, 1] =  focal_length[1]
    intrinsics[2, 0] = principal_point[0]
    intrinsics[2, 1] = principal_point[1]
    return intrinsics


def unpack_pv(payload):
    image    = payload[0:-80]
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

    return _PV_Frame(image, focal_length, principal_point, exposure_time, exposure_compensation, lens_position, focus_state, iso_speed, white_balance, iso_gains, white_balance_gains, resolution)


def get_video_stride(width):
    return (width + 63) & ~63


class _decode_pv:
    def __init__(self, profile):
        self.profile = profile

    def create(self, width, height):
        self._codec = get_video_codec(self.profile)

    def decode(self, payload, format):
        return self._codec.decode(payload).to_ndarray(format=format)


class _unpack_pv:
    _cv2_nv12_format = {
        'rgb24' : cv2.COLOR_YUV2RGB_NV12,
        'bgr24' : cv2.COLOR_YUV2BGR_NV12,
        'rgba'  : cv2.COLOR_YUV2RGBA_NV12,
        'bgra'  : cv2.COLOR_YUV2BGRA_NV12,
        'gray8' : cv2.COLOR_YUV2GRAY_NV12,
        'nv12'  : None
    }

    _resolution = {
        get_video_stride(1952)*1100 : (1952, 1100, get_video_stride(1952)),
        get_video_stride(1504)*846  : (1504,  846, get_video_stride(1504)),
        get_video_stride(1920)*1080 : (1920, 1080, get_video_stride(1920)),
        get_video_stride(1280)*720  : (1280,  720, get_video_stride(1280)),
        get_video_stride(640)*360   : ( 640,  360, get_video_stride( 640)),
        get_video_stride(760)*428   : ( 760,  428, get_video_stride( 760)),
        get_video_stride(960)*540   : ( 960,  540, get_video_stride( 960)),
        get_video_stride(1128)*636  : (1128,  636, get_video_stride(1128)),
        get_video_stride(424)*240   : ( 424,  240, get_video_stride( 424)),
        get_video_stride(500)*282   : ( 500,  282, get_video_stride( 500))
    }

    def create(self, width, height):
        self.width = width
        self.height = height
        self.stride = get_video_stride(width)

    def decode(self, payload, format):
        width, height, stride = _unpack_pv._resolution[(len(payload) * 2) // 3]
        image = np.frombuffer(payload, dtype=np.uint8).reshape(((height*3) //2, stride))[:, :width]
        sf = _unpack_pv._cv2_nv12_format[format]
        return image if (sf is None) else cv2.cvtColor(image, sf)


def decode_pv(profile):
    return _unpack_pv() if (profile == VideoProfile.RAW) else _decode_pv(profile)


#------------------------------------------------------------------------------
# Microphone Decoder
#------------------------------------------------------------------------------

class _decode_microphone:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = get_audio_codec(self.profile)

    def decode(self, payload):
        return self._codec.decode(payload).to_ndarray()


class _unpack_microphone:
    def __init__(self, level):
        self.level = level

    def create(self):
        self.dtype = np.float32 if (self.level == AACLevel.L5) else np.int16

    def decode(self, payload):
        return np.frombuffer(payload, dtype=self.dtype).reshape((1, -1))


def decode_microphone(profile, level):
    return _unpack_microphone(level) if (profile == AudioProfile.RAW) else _decode_microphone(profile)


#------------------------------------------------------------------------------
# SI Unpacker
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


class _SI_Field:
    HEAD  = 1
    EYE   = 2
    LEFT  = 4
    RIGHT = 8


class _SI_HeadPose:
    def __init__(self, position, forward, up):
        self.position = position
        self.forward  = forward
        self.up       = up


class _SI_EyeRay:
    def __init__(self, origin, direction):
        self.origin    = origin
        self.direction = direction


class _SI_HandJointPose:
    def __init__(self, orientation, position, radius, accuracy):
        self.orientation = orientation
        self.position    = position
        self.radius      = radius
        self.accuracy    = accuracy


class _Mode0Layout_SI_Hand:
    BEGIN_ORIENTATION = 0
    END_ORIENTATION   = BEGIN_ORIENTATION + 4*_SIZEOF.FLOAT
    BEGIN_POSITION    = END_ORIENTATION
    END_POSITION      = BEGIN_POSITION + 3*_SIZEOF.FLOAT
    BEGIN_RADIUS      = END_POSITION
    END_RADIUS        = BEGIN_RADIUS + 1*_SIZEOF.FLOAT
    BEGIN_ACCURACY    = END_RADIUS
    END_ACCURACY      = BEGIN_ACCURACY + 1*_SIZEOF.INT
    BYTE_COUNT        = END_ACCURACY


class _Mode0Layout_SI:
    BEGIN_VALID         = 0
    END_VALID           = BEGIN_VALID + 1*_SIZEOF.DWORD
    BEGIN_HEAD_POSITION = END_VALID
    END_HEAD_POSITION   = BEGIN_HEAD_POSITION + 3*_SIZEOF.FLOAT
    BEGIN_HEAD_FORWARD  = END_HEAD_POSITION
    END_HEAD_FORWARD    = BEGIN_HEAD_FORWARD + 3*_SIZEOF.FLOAT
    BEGIN_HEAD_UP       = END_HEAD_FORWARD
    END_HEAD_UP         = BEGIN_HEAD_UP + 3*_SIZEOF.FLOAT
    BEGIN_EYE_ORIGIN    = END_HEAD_UP
    END_EYE_ORIGIN      = BEGIN_EYE_ORIGIN + 3*_SIZEOF.FLOAT
    BEGIN_EYE_DIRECTION = END_EYE_ORIGIN
    END_EYE_DIRECTION   = BEGIN_EYE_DIRECTION + 3*_SIZEOF.FLOAT
    BEGIN_HAND_LEFT     = END_EYE_DIRECTION
    END_HAND_LEFT       = BEGIN_HAND_LEFT + SI_HandJointKind.TOTAL * _Mode0Layout_SI_Hand.BYTE_COUNT
    BEGIN_HAND_RIGHT    = END_HAND_LEFT
    END_HAND_RIGHT      = BEGIN_HAND_RIGHT + SI_HandJointKind.TOTAL * _Mode0Layout_SI_Hand.BYTE_COUNT


class _SI_Hand:
    def __init__(self, payload):
        self._data = payload

    def get_joint_pose(self, joint):
        begin = joint * _Mode0Layout_SI_Hand.BYTE_COUNT
        end = begin + _Mode0Layout_SI_Hand.BYTE_COUNT
        data = self._data[begin:end]

        orientation = np.frombuffer(data[_Mode0Layout_SI_Hand.BEGIN_ORIENTATION : _Mode0Layout_SI_Hand.END_ORIENTATION], dtype=np.float32)
        position    = np.frombuffer(data[_Mode0Layout_SI_Hand.BEGIN_POSITION    : _Mode0Layout_SI_Hand.END_POSITION],    dtype=np.float32)
        radius      = np.frombuffer(data[_Mode0Layout_SI_Hand.BEGIN_RADIUS      : _Mode0Layout_SI_Hand.END_RADIUS],      dtype=np.float32)
        accuracy    = np.frombuffer(data[_Mode0Layout_SI_Hand.BEGIN_ACCURACY    : _Mode0Layout_SI_Hand.END_ACCURACY],    dtype=np.int32)

        return _SI_HandJointPose(orientation, position, radius, accuracy)


class unpack_si:
    def __init__(self, payload):
        self._data = payload
        self._valid = np.frombuffer(payload[_Mode0Layout_SI.BEGIN_VALID : _Mode0Layout_SI.END_VALID], dtype=np.uint32)

    def is_valid_head_pose(self):
        return (self._valid & _SI_Field.HEAD) != 0

    def is_valid_eye_ray(self):
        return (self._valid & _SI_Field.EYE) != 0

    def is_valid_hand_left(self):
        return (self._valid & _SI_Field.LEFT) != 0

    def is_valid_hand_right(self):
        return (self._valid & _SI_Field.RIGHT) != 0

    def get_head_pose(self):
        position = np.frombuffer(self._data[_Mode0Layout_SI.BEGIN_HEAD_POSITION : _Mode0Layout_SI.END_HEAD_POSITION], dtype=np.float32)
        forward  = np.frombuffer(self._data[_Mode0Layout_SI.BEGIN_HEAD_FORWARD  : _Mode0Layout_SI.END_HEAD_FORWARD],  dtype=np.float32)
        up       = np.frombuffer(self._data[_Mode0Layout_SI.BEGIN_HEAD_UP       : _Mode0Layout_SI.END_HEAD_UP],       dtype=np.float32)

        return _SI_HeadPose(position, forward, up)

    def get_eye_ray(self):
        origin    = np.frombuffer(self._data[_Mode0Layout_SI.BEGIN_EYE_ORIGIN    : _Mode0Layout_SI.END_EYE_ORIGIN],    dtype=np.float32)
        direction = np.frombuffer(self._data[_Mode0Layout_SI.BEGIN_EYE_DIRECTION : _Mode0Layout_SI.END_EYE_DIRECTION], dtype=np.float32)

        return _SI_EyeRay(origin, direction)

    def get_hand_left(self):
        return _SI_Hand(self._data[_Mode0Layout_SI.BEGIN_HAND_LEFT : _Mode0Layout_SI.END_HAND_LEFT])

    def get_hand_right(self):
        return _SI_Hand(self._data[_Mode0Layout_SI.BEGIN_HAND_RIGHT : _Mode0Layout_SI.END_HAND_RIGHT])


#------------------------------------------------------------------------------
# EET Unpacker
#------------------------------------------------------------------------------

class unpack_eet:
    def __init__(self, payload):
        self._reserved = payload[:4]
        f = np.frombuffer(payload[4:-4], dtype=np.float32)
        valid = struct.unpack('<I', payload[-4:])[0]

        self.combined_ray = _SI_EyeRay(f[0:3], f[3:6])
        self.left_ray = _SI_EyeRay(f[6:9], f[9:12])
        self.right_ray = _SI_EyeRay(f[12:15], f[15:18])
        self.left_openness = f[18]
        self.right_openness = f[19]
        self.vergence_distance = f[20]

        self.calibration_valid = valid & 0x01 != 0
        self.combined_ray_valid = valid & 0x02 != 0
        self.left_ray_valid = valid & 0x04 != 0
        self.right_ray_valid = valid & 0x08 != 0
        self.left_openness_valid = valid & 0x10 != 0
        self.right_openness_valid = valid & 0x20 != 0
        self.vergence_distance_valid = valid & 0x40 != 0


#------------------------------------------------------------------------------
# Extended Depth Decoder
#------------------------------------------------------------------------------

class _EZ_Frame:
    def __init__(self, depth, width, height):
        self.depth  = depth
        self.width  = width
        self.height = height


def unpack_extended_depth(payload):
    width, height = struct.unpack('<HH', payload[-4:])
    return _EZ_Frame(payload[:-4], width, height)


class _unpack_extended_depth:
    def create(self):
        pass

    def decode(self, payload, width, height):
        return np.frombuffer(payload, dtype=np.uint16).reshape((height, width))


class _decode_extended_depth:
    def create(self):
        import pyzdepth
        self._codec = pyzdepth.DepthCompressor()

    def decode(self, payload, width, height):
        if (len(payload) <= 0):
            return None
        result, width, height, decompressed = self._codec.Decompress(bytes(payload))
        return np.frombuffer(decompressed, dtype=np.uint16).reshape((height, width))


def decode_extended_depth(profile_z):
    return _decode_extended_depth() if (profile_z == DepthProfile.ZDEPTH) else _unpack_extended_depth()


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_rm_vlc(rx_rm_vlc):
    def __init__(self, host, port, chunk, mode, divisor, profile, level, bitrate, options):
        super().__init__(host, port, chunk, mode, divisor, profile, level, bitrate, options)
        self._codec = decode_rm_vlc(profile)

    def open(self):
        self._codec.create()
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = unpack_rm_vlc(data.payload)
        data.payload.image = self._codec.decode(data.payload.image)
        return data

    def close(self):
        super().close()


class rx_decoded_rm_depth_ahat(rx_rm_depth_ahat):
    def __init__(self, host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options):
        super().__init__(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)
        self._codec = decode_rm_depth_ahat(profile_z, profile_ab)

    def open(self):
        self._codec.create()
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_rm_depth_longthrow(rx_rm_depth_longthrow):
    def __init__(self, host, port, chunk, mode, divisor, png_filter):
        super().__init__(host, port, chunk, mode, divisor, png_filter)

    def open(self):
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = decode_rm_depth_longthrow(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_pv(rx_pv):
    def __init__(self, host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options, format):
        super().__init__(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)
        self.format = format
        self._codec = decode_pv(profile)

    def open(self):        
        self._codec.create(self.width, self.height)
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = unpack_pv(data.payload)
        data.payload.image = self._codec.decode(data.payload.image, self.format)
        return data

    def close(self):
        super().close()


class rx_decoded_microphone(rx_microphone):
    def __init__(self, host, port, chunk, profile, level):
        super().__init__(host, port, chunk, profile, level)
        self._codec = decode_microphone(profile, level)
        
    def open(self):
        self._codec.create()
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_extended_audio(rx_extended_audio):
    def __init__(self, host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level):
        super().__init__(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level)
        self._codec = decode_microphone(profile, None)
        
    def open(self):
        self._codec.create()
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_extended_depth(rx_extended_depth):
    def __init__(self, host, port, chunk, mode, divisor, profile_z, options):
        super().__init__(host, port, chunk, mode, divisor, profile_z, options)
        self._codec = decode_extended_depth(profile_z)

    def open(self):
        self._codec.create()
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = unpack_extended_depth(data.payload)
        data.payload.depth = self._codec.decode(data.payload.depth, data.payload.width, data.payload.height)
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


def _download_mode2_data(host, port, configuration, bytes):
    c = _client()

    c.open(host, port)
    c.sendall(configuration)
    data = c.download(bytes, ChunkSize.SINGLE_TRANSFER)
    c.close()

    return data


def download_calibration_rm_vlc(host, port):
    data   = _download_mode2_data(host, port, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_VLC.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    uv2x       = floats[_Mode2Layout_RM_VLC.BEGIN_UV2X       : _Mode2Layout_RM_VLC.END_UV2X      ].reshape(Parameters_RM_VLC.SHAPE)
    uv2y       = floats[_Mode2Layout_RM_VLC.BEGIN_UV2Y       : _Mode2Layout_RM_VLC.END_UV2Y      ].reshape(Parameters_RM_VLC.SHAPE)
    extrinsics = floats[_Mode2Layout_RM_VLC.BEGIN_EXTRINSICS : _Mode2Layout_RM_VLC.END_EXTRINSICS].reshape((4, 4))
    mapx       = floats[_Mode2Layout_RM_VLC.BEGIN_MAPX       : _Mode2Layout_RM_VLC.END_MAPX      ].reshape(Parameters_RM_VLC.SHAPE)
    mapy       = floats[_Mode2Layout_RM_VLC.BEGIN_MAPY       : _Mode2Layout_RM_VLC.END_MAPY      ].reshape(Parameters_RM_VLC.SHAPE)
    k          = floats[_Mode2Layout_RM_VLC.BEGIN_K          : _Mode2Layout_RM_VLC.END_K         ]

    intrinsics = np.array([[k[0], 0, 0, 0], [0, k[1], 0, 0], [k[2], k[3], 1, 0], [0, 0, 0, 1]], dtype=np.float32)
    
    return _Mode2_RM_VLC(np.dstack((uv2x, uv2y)), extrinsics, np.dstack((mapx, mapy)), intrinsics)


def download_calibration_rm_depth_ahat(host, port):
    data   = _download_mode2_data(host, port, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_DEPTH_AHAT.FLOAT_COUNT * _SIZEOF.FLOAT)
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


def download_calibration_rm_depth_longthrow(host, port):
    data   = _download_mode2_data(host, port, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_DEPTH_LONGTHROW.FLOAT_COUNT * _SIZEOF.FLOAT)
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


def download_calibration_rm_imu(host, port):
    data   = _download_mode2_data(host, port, _create_configuration_for_rm_mode2(StreamMode.MODE_2), _Mode2Layout_RM_IMU.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    extrinsics = floats[_Mode2Layout_RM_IMU.BEGIN_EXTRINSICS : _Mode2Layout_RM_IMU.END_EXTRINSICS].reshape((4, 4))

    return _Mode2_RM_IMU(extrinsics)


def download_calibration_pv(host, port, width, height, framerate):
    data   = _download_mode2_data(host, port, _create_configuration_for_pv_mode2(StreamMode.MODE_2, width, height, framerate), _Mode2Layout_PV.FLOAT_COUNT * _SIZEOF.FLOAT)
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


def download_devicelist_extended_audio(host, port):
    c = _client()
    c.open(host, port)
    c.sendall(_create_configuration_for_mrc_audio(MixerMode.QUERY, 1.0, 1.0))
    size = struct.unpack('<I', c.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))[0]
    query = c.download(size, ChunkSize.SINGLE_TRANSFER).decode('utf-16')
    c.close()
    return query


def download_devicelist_extended_video(host, port):
    c = _client()
    c.open(host, port)
    c.sendall(_create_configuration_for_mode(StreamMode.MODE_2))
    size = struct.unpack('<I', c.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))[0]
    query = c.download(size, ChunkSize.SINGLE_TRANSFER).decode('utf-16')
    c.close()
    return query


#------------------------------------------------------------------------------
# Port Information
#------------------------------------------------------------------------------

class _PortName:
    OF = [
        'rm_vlc_leftfront',
        'rm_vlc_leftleft',
        'rm_vlc_rightfront', 
        'rm_vlc_rightright', 
        'rm_depth_ahat', 
        'rm_depth_longthrow', 
        'rm_imu_accelerometer', 
        'rm_imu_gyroscope', 
        'rm_imu_magnetometer', 
        'remote_configuration', 
        'personal_video', 
        'microphone', 
        'spatial_input', 
        'spatial_mapping', 
        'scene_understanding',
        'voice_input',
        'unity_message_queue',
        'extended_eye_tracker',
        'extended_audio',
        'extended_video',
        'guest_message_queue',
        'extended_depth',
    ]


def get_port_name(port):
    return _PortName.OF[port - StreamPort.RM_VLC_LEFTFRONT]


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


class PV_RegionOfInterestType:
    Unknown = 0
    Face = 1

class InterfacePriority:
    LOWEST = -2
    BELOW_NORMAL = -1
    NORMAL = 0
    ABOVE_NORMAL = 1
    HIGHEST = 2

class ipc_rc(_context_manager):
    _CMD_GET_APPLICATION_VERSION = 0x00
    _CMD_GET_UTC_OFFSET = 0x01
    _CMD_SET_HS_MARKER_STATE = 0x02
    _CMD_GET_PV_SUBSYSTEM_STATUS = 0x03
    _CMD_SET_PV_FOCUS = 0x04
    _CMD_SET_PV_VIDEO_TEMPORAL_DENOISING = 0x05
    _CMD_SET_PV_WHITE_BALANCE_PRESET = 0x06
    _CMD_SET_PV_WHITE_BALANCE_VALUE = 0x07
    _CMD_SET_PV_EXPOSURE = 0x08
    _CMD_SET_PV_EXPOSURE_PRIORITY_VIDEO = 0x09
    _CMD_SET_PV_ISO_SPEED = 0x0A
    _CMD_SET_PV_BACKLIGHT_COMPENSATION = 0x0B
    _CMD_SET_PV_SCENE_MODE = 0x0C
    _CMD_SET_FLAT_MODE = 0x0D
    _CMD_SET_RM_EYE_SELECTION = 0x0E
    _CMD_SET_PV_DESIRED_OPTIMIZATION = 0x0F
    _CMD_SET_PV_PRIMARY_USE = 0x10
    _CMD_SET_PV_OPTICAL_IMAGE_STABILIZATION = 0x11
    _CMD_SET_PV_HDR_VIDEO = 0x12
    _CMD_SET_PV_REGIONS_OF_INTEREST = 0x13
    _CMD_SET_INTERFACE_PRIORITY = 0x14
    _CMD_SET_QUIET_MODE = 0x15

    def __init__(self, host, port):
        self.host = host
        self.port = port

    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

    def close(self):
        self._client.close()

    def ee_get_application_version(self):
        command = struct.pack('<B', ipc_rc._CMD_GET_APPLICATION_VERSION)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.SHORT * 4, ChunkSize.SINGLE_TRANSFER)
        version = struct.unpack('<HHHH', data)
        return version

    def ts_get_utc_offset(self):
        command = struct.pack('<B', ipc_rc._CMD_GET_UTC_OFFSET)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.LONGLONG, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<Q', data)[0]

    def hs_set_marker_state(self, state):
        command = struct.pack('<BI', ipc_rc._CMD_SET_HS_MARKER_STATE, state)
        self._client.sendall(command)

    def pv_get_subsystem_status(self):
        command = struct.pack('<B', ipc_rc._CMD_GET_PV_SUBSYSTEM_STATUS)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.BYTE, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<B', data)[0] != 0
    
    def pv_wait_for_subsystem(self, status):
        while (self.pv_get_subsystem_status() != status):
            pass

    def pv_set_focus(self, focusmode, autofocusrange, distance, value, driverfallback):
        command = struct.pack('<BIIIII', ipc_rc._CMD_SET_PV_FOCUS, focusmode, autofocusrange, distance, value, driverfallback)
        self._client.sendall(command)

    def pv_set_video_temporal_denoising(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_VIDEO_TEMPORAL_DENOISING, mode)
        self._client.sendall(command)

    def pv_set_white_balance_preset(self, preset):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_WHITE_BALANCE_PRESET, preset)
        self._client.sendall(command)

    def pv_set_white_balance_value(self, value):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_WHITE_BALANCE_VALUE, value)
        self._client.sendall(command)

    def pv_set_exposure(self, mode, value):
        command = struct.pack('<BII', ipc_rc._CMD_SET_PV_EXPOSURE, mode, value)
        self._client.sendall(command)
    
    def pv_set_exposure_priority_video(self, enabled):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_EXPOSURE_PRIORITY_VIDEO, enabled)
        self._client.sendall(command)

    def pv_set_iso_speed(self, mode, value):
        command = struct.pack('<BII', ipc_rc._CMD_SET_PV_ISO_SPEED, mode, value)
        self._client.sendall(command)

    def pv_set_backlight_compensation(self, state):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_BACKLIGHT_COMPENSATION, state)
        self._client.sendall(command)

    def pv_set_scene_mode(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_SCENE_MODE, mode)
        self._client.sendall(command)

    def ee_set_flat_mode(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_FLAT_MODE, mode)
        self._client.sendall(command)

    def rm_set_eye_selection(self, enable):
        command = struct.pack('<BI', ipc_rc._CMD_SET_RM_EYE_SELECTION, 1 if (enable) else 0)
        self._client.sendall(command)

    def pv_set_desired_optimization(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_DESIRED_OPTIMIZATION, mode)
        self._client.sendall(command)

    def pv_set_primary_use(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_PRIMARY_USE, mode)
        self._client.sendall(command)

    def pv_set_optical_image_stabilization(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_OPTICAL_IMAGE_STABILIZATION, mode)
        self._client.sendall(command)

    def pv_set_hdr_video(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_PV_HDR_VIDEO, mode)
        self._client.sendall(command)

    def pv_set_regions_of_interest(self, clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h):
        mode = (0x1000 if (clear) else 0) | (0x0800 if (set) else 0) | (0x0400 if (auto_exposure) else 0) | (0x0200 if (auto_focus) else 0) | (0x0100 if (bounds_normalized) else 0) | ((type & 1) << 7) | (weight & 0x007F)
        command = struct.pack('<BIffff', ipc_rc._CMD_SET_PV_REGIONS_OF_INTEREST, mode, x, y, w, h)
        self._client.sendall(command)

    def ee_set_interface_priority(self, port, priority):
        command = struct.pack('<BIi', ipc_rc._CMD_SET_INTERFACE_PRIORITY, port, priority)
        self._client.sendall(command)

    def ee_set_quiet_mode(self, mode):
        command = struct.pack('<BI', ipc_rc._CMD_SET_QUIET_MODE, mode)
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

    def add_task(self, id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format):
        self._data.extend(struct.pack('<16sdIIII', id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format, 0))
        self._count += 1

    def _get(self):
        return self._count, self._data


class _sm_mesh:
    def __init__(self, vertex_position_scale, pose, bounds, vertex_positions, triangle_indices, vertex_normals):
        self.vertex_position_scale = vertex_position_scale
        self.pose                  = pose
        self.bounds                = bounds        
        self.vertex_positions      = vertex_positions
        self.triangle_indices      = triangle_indices
        self.vertex_normals        = vertex_normals

    def unpack(self, vertex_position_format, triangle_index_format, vertex_normal_format):
        self.vertex_position_scale = np.frombuffer(self.vertex_position_scale, dtype=np.float32).reshape((1, 3))
        self.pose                  = np.frombuffer(self.pose,                  dtype=np.float32).reshape((4, 4))
        self.bounds                = np.frombuffer(self.bounds,                dtype=np.float32)        
        self.vertex_positions      = np.frombuffer(self.vertex_positions,      dtype=_SM_Convert.DirectXPixelFormatToNumPy[vertex_position_format]).reshape((-1, 4))
        self.triangle_indices      = np.frombuffer(self.triangle_indices,      dtype=_SM_Convert.DirectXPixelFormatToNumPy[triangle_index_format]).reshape((-1, 3))
        self.vertex_normals        = np.frombuffer(self.vertex_normals,        dtype=_SM_Convert.DirectXPixelFormatToNumPy[vertex_normal_format]).reshape((-1, 4))


class ipc_sm(_context_manager):
    _CMD_SET_VOLUMES           = 0x00
    _CMD_GET_OBSERVED_SURFACES = 0x01
    _CMD_GET_MESHES            = 0x02

    def __init__(self, host, port):
        self.host = host
        self.port = port
        
    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

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
    
    def _download_mesh(self):
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

        return index, _sm_mesh(scale, pose, bounds, vertex_positions, triangle_indices, vertex_normals)
    
    def _download_meshes(self, count):
        for _ in range(0, count):
            yield self._download_mesh()
    
    def get_meshes(self, tasks):
        count, data = tasks._get()
        msg = bytearray()
        msg.extend(struct.pack('<BI', ipc_sm._CMD_GET_MESHES, count))
        msg.extend(data)
        self._client.sendall(msg)
        meshes = {index : mesh for index, mesh in self._download_meshes(count)}
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

    def pack(self):
        self._task = bytearray()
        self._task.extend(struct.pack('<BBBBIfBBBBBBBBI', self.enable_quads, self.enable_meshes, self.enable_only_observed, self.enable_world_mesh, self.mesh_lod, self.query_radius, self.create_mode, self.kind_flags, self.get_orientation, self.get_position, self.get_location_matrix, self.get_quad, self.get_meshes, self.get_collider_meshes, len(self.guid_list)))
        for guid in self.guid_list:
            self._task.extend(guid)

    def _get(self):
        return self._task


class _su_mesh:
    def __init__(self, vertex_positions, triangle_indices):
        self.vertex_positions = vertex_positions
        self.triangle_indices = triangle_indices

    def unpack(self):
        self.vertex_positions = np.frombuffer(self.vertex_positions, dtype=np.float32).reshape((-1, 3))
        self.triangle_indices = np.frombuffer(self.triangle_indices, dtype=np.uint32).reshape((-1, 3))


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

    def unpack(self):
        self.kind = np.frombuffer(self.kind, dtype=np.int32)
        self.orientation = np.frombuffer(self.orientation, dtype=np.float32)
        self.position = np.frombuffer(self.position, dtype=np.float32)
        self.location = np.frombuffer(self.location, dtype=np.float32).reshape((-1, 4))
        self.alignment = np.frombuffer(self.alignment, dtype=np.int32)
        self.extents = np.frombuffer(self.extents, dtype=np.float32)


class _su_result:
    def __init__(self, extrinsics, pose, items):        
        self.extrinsics = extrinsics
        self.pose = pose
        self.items = items

    def unpack(self):
        self.extrinsics = np.frombuffer(self.extrinsics, dtype=np.float32).reshape((4, 4))
        self.pose = np.frombuffer(self.pose, dtype=np.float32).reshape((4, 4))


class ipc_su(_context_manager):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        
    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

    def _download_mesh(self):
        elements_vertices, elements_indices = struct.unpack('<II', self._client.download(2 * _SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))
        vpl = elements_vertices * _SIZEOF.DWORD
        til = elements_indices * _SIZEOF.DWORD
        data = self._client.download(vpl + til, ChunkSize.SINGLE_TRANSFER)
        return _su_mesh(data[:vpl], data[vpl:])

    def _download_meshes(self):
        return [self._download_mesh() for _ in range(0, struct.unpack('<I', self._client.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER))[0])]
    
    def _download_item(self, bi, bk, bo, bp, bl, ba, be, bm, download_meshes, download_collider_meshes):
        d = self._client.download(bm, ChunkSize.SINGLE_TRANSFER)
        return _su_item(d[bi:bk], d[bk:bo], d[bo:bp], d[bp:bl], d[bl:ba], d[ba:be], d[be:bm], self._download_meshes() if (download_meshes) else [], self._download_meshes() if (download_collider_meshes) else [])
    
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
        return _su_result(header[he:hp], header[hp:hi], [self._download_item(bi, bk, bo, bp, bl, ba, be, bm, task.get_meshes, task.get_collider_meshes) for _ in range(0, struct.unpack('<I', header[132:])[0])])

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


class vi_result:
    def __init__(self, index, confidence, phrase_duration, phrase_start_time, raw_confidence):
        self.index = index
        self.confidence = confidence
        self.phrase_duration = phrase_duration
        self.phrase_start_time = phrase_start_time
        self.raw_confidence = raw_confidence

    def unpack(self):
        self.index = struct.unpack('<I', self.index)[0]
        self.confidence = struct.unpack('<I', self.confidence)[0]
        self.phrase_duration = struct.unpack('<Q', self.phrase_duration)[0]
        self.phrase_start_time = struct.unpack('<Q', self.phrase_start_time)[0]
        self.raw_confidence = struct.unpack('<d', self.raw_confidence)[0]


class ipc_vi(_context_manager):
    _CMD_POP  = 0x01
    _CMD_STOP = 0x00

    def __init__(self, host, port):
        self.host = host
        self.port = port

    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

    def start(self, strings):
        command = bytearray()
        command.extend(struct.pack('<H', len(strings)))
        for string in strings:
            encoded = string.encode('utf-16')
            command.extend(struct.pack('<H', len(encoded)))
            command.extend(encoded)
        self._client.sendall(command)

    def pop(self):
        command = struct.pack('<B', ipc_vi._CMD_POP)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER)
        count = struct.unpack('<I', data)[0]
        data = self._client.download(32*count, ChunkSize.SINGLE_TRANSFER)
        return [vi_result(data[(i*32):(i*32+4)], data[(i*32+4):(i*32+8)], data[(i*32+8):(i*32+16)], data[(i*32+16):(i*32+24)], data[(i*32+24):(i*32+32)]) for i in range(0, count)]

    def stop(self):
        command = struct.pack('<B', ipc_vi._CMD_STOP)
        self._client.sendall(command)

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
    def __init__(self, host, port):
        self.host = host
        self.port = port
    
    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

    def push(self, buffer):
        self._client.sendall(buffer.get_data())

    def pull(self, buffer):
        return self.pull_n(buffer.get_count())
    
    def pull_n(self, count):
        return np.frombuffer(self._client.download(_SIZEOF.DWORD * count, ChunkSize.SINGLE_TRANSFER), dtype=np.uint32)

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Guest Message Queue
#------------------------------------------------------------------------------

class ipc_gmq(_context_manager):
    _CMD_NONE = _RANGEOF.U32_MAX

    def __init__(self, host, port):
        self.host = host
        self.port = port

    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

    def pull(self):
        self._client.sendall(struct.pack('<I', ipc_gmq._CMD_NONE))
        header = struct.unpack('<II', self._client.download(_SIZEOF.DWORD * 2, ChunkSize.SINGLE_TRANSFER))
        data = self._client.download(header[1], ChunkSize.SINGLE_TRANSFER) if (header[1] > 0) else b''
        return (header[0], data) if (header[0] != ipc_gmq._CMD_NONE) else None
    
    def push(self, response):
        self._client.sendall(struct.pack('<I', response))
    
    def close(self):
        self._client.close()

