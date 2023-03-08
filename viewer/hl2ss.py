
import numpy as np
import socket
import struct
import asyncio
import websockets.client
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


# IPC TCP Ports
class IPCPort:
    REMOTE_CONFIGURATION = 3809
    SPATIAL_MAPPING      = 3813
    SCENE_UNDERSTANDING  = 3814


# Default Chunk Sizes
class ChunkSize:
    RM_VLC               = 4096
    RM_DEPTH_AHAT        = 4096
    RM_DEPTH_LONGTHROW   = 4096
    RM_IMU_ACCELEROMETER = 2048
    RM_IMU_GYROSCOPE     = 4096
    RM_IMU_MAGNETOMETER  = 256
    PERSONAL_VIDEO       = 4096
    MICROPHONE           = 512
    SPATIAL_INPUT        = 1024
    SINGLE_TRANSFER      = 4096


# Stream Operating Mode
# 0: device data (e.g. video)
# 1: device data + location data (e.g. video + camera pose)
# 2: device constants (e.g. camera intrinsics)
class StreamMode:
    MODE_0 = 0
    MODE_1 = 1
    MODE_2 = 2


# Video Encoder Configuration
# 0: H264 base
# 1: H264 main
# 2: H264 high
# 3: H265 main (HEVC)
class VideoProfile:
    H264_BASE = 0
    H264_MAIN = 1
    H264_HIGH = 2
    H265_MAIN = 3
    RAW       = 0xFF


# Audio Encoder Configuration
# 0: AAC 12000 bytes/s
# 1: AAC 16000 bytes/s
# 2: AAC 20000 bytes/s
# 3: AAC 24000 bytes/s
class AudioProfile:
    AAC_12000 = 0
    AAC_16000 = 1
    AAC_20000 = 2
    AAC_24000 = 3
    RAW       = 0xFF


# PNG Filters
class PngFilterMode:
    Automatic = 0
    Disable   = 1
    Sub       = 2
    Up        = 3
    Average   = 4
    Paeth     = 5
    Adaptive  = 6


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
    FACTOR = 4


# RM Depth Long Throw Parameters
class Parameters_RM_DEPTH_LONGTHROW:
    WIDTH  = 320
    HEIGHT = 288
    FPS    = 5
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)
    PERIOD = 1 / FPS


# Microphone Parameters
class Parameters_MICROPHONE:
    SAMPLE_RATE = 48000
    GROUP_SIZE  = 1024
    CHANNELS    = 2
    PERIOD      = GROUP_SIZE / SAMPLE_RATE


# Spatial Input Parameters
class Parameters_SI:
    SAMPLE_RATE = 60
    PERIOD      = 1 / SAMPLE_RATE


# Time base for all timestamps
class TimeBase:
    HUNDREDS_OF_NANOSECONDS = 10*1000*1000


# Hand Joints
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


# Display Marker
class HS_MarkerState:
    Disable = 0
    Enable = 1


# Focus Modes
class PV_FocusMode:
    Auto = 0
    Single = 1
    Continuous = 2
    Manual = 3


# Auto Focus Range
class PV_AutoFocusRange:
    FullRange = 0
    Macro = 1
    Normal = 2


# Manual Focus Distance
class PV_ManualFocusDistance:
    Infinity = 0
    Nearest = 2


# Minimum and maximum allowed focus values
class PV_FocusValue:
    Min = 170
    Max = 10000


# Driver configuration for auto focus
class PV_DriverFallback:
    Enable = 0
    Disable = 1


# Video Temporal Denoising
class PV_VideoTemporalDenoisingMode:
    Off = 0
    On = 1


# White Balance Presets
class PV_ColorTemperaturePreset:
    Auto = 0
    Manual = 1
    Cloudy = 2
    Daylight = 3
    Flash = 4
    Fluorescent = 5
    Tungsten = 6
    Candlelight = 7


# Minimum and maximum allowed values for white balance
class PV_WhiteBalanceValue:
    Min = 2300 // 25
    Max = 7500 // 25


# Exposure mode
class PV_ExposureMode:
    Manual = 0
    Auto = 1
    

# Minimum and maximum allowed values for exposure
class PV_ExposureValue:
    Min = 1000 // 10
    Max = 660000 // 10


# Exposure priority video mode
class PV_ExposurePriorityVideo:
    Disabled = 0
    Enabled = 1


# Capture scene mode
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


# Iso speed mode
class PV_IsoSpeedMode:
    Manual = 0
    Auto = 1


# Minimum and maximum allowed values for iso speed
class PV_IsoSpeedValue:
    Min = 100
    Max = 3200


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

def _create_configuration_for_mode(mode):
    return struct.pack('<B', mode)


def _create_configuration_for_video_format(width, height, framerate):
    return struct.pack('<HHB', width, height, framerate)


def _create_configuration_for_video_encoding(profile, bitrate):
    return struct.pack('<BI', profile, bitrate)


def _create_configuration_for_audio_encoding(profile):
    return struct.pack('<B', profile)


def _create_configuration_for_png_encoding(png_filter):
    return struct.pack('<B', png_filter)


def _create_configuration_for_rm_vlc(mode, profile, bitrate):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_encoding(profile, bitrate))
    return bytes(configuration)


def _create_configuration_for_rm_depth_ahat(mode, profile, bitrate):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_encoding(profile, bitrate))
    return bytes(configuration)


def _create_configuration_for_rm_depth_longthrow(mode, png_filter):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_png_encoding(png_filter))
    return bytes(configuration)


def _create_configuration_for_rm_imu(mode):
    return _create_configuration_for_mode(mode)


def _create_configuration_for_pv(mode, width, height, framerate, profile, bitrate):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_format(width, height, framerate))
    configuration.extend(_create_configuration_for_video_encoding(profile, bitrate))
    return bytes(configuration)


def _create_configuration_for_microphone(profile):
    return _create_configuration_for_audio_encoding(profile)


def _create_configuration_for_rm_mode2(mode):
    return _create_configuration_for_mode(mode)


def _create_configuration_for_pv_mode2(mode, width, height, framerate):
    configuration = bytearray()
    configuration.extend(_create_configuration_for_mode(mode))
    configuration.extend(_create_configuration_for_video_format(width, height, framerate))
    return bytes(configuration)


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Acquisition
#------------------------------------------------------------------------------

def _connect_client_rm_vlc(host, port, chunk_size, mode, profile, bitrate):
    if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)
    else:
        c = _gatherer()
        c.open(host, port, chunk_size, mode)
        c.sendall(_create_configuration_for_rm_vlc(mode, profile, bitrate))
    return c


def _connect_client_rm_depth_ahat(host, port, chunk_size, mode, profile, bitrate):
    if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)
    else:
        c = _gatherer()
        c.open(host, port, chunk_size, mode)
        c.sendall(_create_configuration_for_rm_depth_ahat(mode, profile, bitrate))
    return c


def _connect_client_rm_depth_longthrow(host, port, chunk_size, mode, png_filter):
    if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)
    else:
        c = _gatherer()
        c.open(host, port, chunk_size, mode)
        c.sendall(_create_configuration_for_rm_depth_longthrow(mode, png_filter))
    return c


def _connect_client_rm_imu(host, port, chunk_size, mode):
    if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)
    else:
        c = _gatherer()
        c.open(host, port, chunk_size, mode)
        c.sendall(_create_configuration_for_rm_imu(mode))
    return c


def _connect_client_pv(host, port, chunk_size, mode, width, height, framerate, profile, bitrate):
    if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)
    else:
        c = _gatherer()
        c.open(host, port, chunk_size, mode)
        c.sendall(_create_configuration_for_pv(mode, width, height, framerate, profile, bitrate))
    return c


def _connect_client_microphone(host, port, chunk_size, profile):
    if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)
    else:
        c = _gatherer()
        c.open(host, port, chunk_size, StreamMode.MODE_0)
        c.sendall(_create_configuration_for_microphone(profile))
    return c


def _connect_client_si(host, port, chunk_size):
    if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)
    else:
        c = _gatherer()
        c.open(host, port, chunk_size, StreamMode.MODE_0)
    return c


def start_subsystem_pv(host, port):
    if (is_rs_host(host)):
        return
    c = _client()
    c.open(host, port)
    c.sendall(_create_configuration_for_pv_mode2(0x7, 1920, 1080, 30))
    c.close()


def stop_subsystem_pv(host, port):
    if (is_rs_host(host)):
        return
    c = _client()
    c.open(host, port)
    c.sendall(_create_configuration_for_pv_mode2(0xB, 1920, 1080, 30))
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
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.profile = profile
        self.bitrate = bitrate

    def open(self):
        self._client = _connect_client_rm_vlc(self.host, self.port, self.chunk, self.mode, self.profile, self.bitrate)

    def get_next_packet(self):
        data = self._client.get_next_packet()
        if (self.profile == VideoProfile.RAW):
            self._client.sendall(b'\x00')
        return data

    def close(self):
        self._client.close()


class rx_rm_depth_ahat(_context_manager):
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.profile = profile
        self.bitrate = bitrate

    def open(self):
        self._client = _connect_client_rm_depth_ahat(self.host, self.port, self.chunk, self.mode, self.profile, self.bitrate)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_rm_depth_longthrow(_context_manager):
    def __init__(self, host, port, chunk, mode, png_filter):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.png_filter = png_filter

    def open(self):
        self._client = _connect_client_rm_depth_longthrow(self.host, self.port, self.chunk, self.mode, self.png_filter)

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
    def __init__(self, host, port, chunk, mode, width, height, framerate, profile, bitrate):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = mode
        self.width = width
        self.height = height
        self.framerate = framerate
        self.profile = profile
        self.bitrate = bitrate

    def open(self):
        self._client = _connect_client_pv(self.host, self.port, self.chunk, self.mode, self.width, self.height, self.framerate, self.profile, self.bitrate)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_microphone(_context_manager):
    def __init__(self, host, port, chunk, profile):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = StreamMode.MODE_0
        self.profile = profile

    def open(self):
        self._client = _connect_client_microphone(self.host, self.port, self.chunk, self.profile)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_si(_context_manager):
    def __init__(self, host, port, chunk):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.mode = StreamMode.MODE_0

    def open(self):
        self._client = _connect_client_si(self.host, self.port, self.chunk)

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


def get_gop_size(profile, framerate):
    name = get_video_codec_name(profile)
    return 1 if ((name != 'h264') and (name != 'hevc')) else (2 * framerate)


def get_video_codec_default_factor(profile):
    name = get_video_codec_name(profile)
    return 4/420 if (name == 'h264') else 1/140 if (name == 'hevc') else 1.0


def get_video_codec_default_bitrate(width, height, fps, profile):
    return int(width*height*fps*12*get_video_codec_default_factor(profile))


#------------------------------------------------------------------------------
# RM VLC Decoder
#------------------------------------------------------------------------------

class decode_rm_vlc:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = av.CodecContext.create(get_video_codec_name(self.profile), 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return frame.to_ndarray()[:Parameters_RM_VLC.HEIGHT, :Parameters_RM_VLC.WIDTH]
        return None


#------------------------------------------------------------------------------
# RM Depth Decoder
#------------------------------------------------------------------------------

class _RM_Depth_Frame:
    def __init__(self, depth, ab):
        self.depth = depth
        self.ab    = ab


class _Mode0Layout_RM_DEPTH_AHAT:
    BEGIN_DEPTH_Y = 0
    END_DEPTH_Y   = BEGIN_DEPTH_Y + Parameters_RM_DEPTH_AHAT.HEIGHT
    BEGIN_AB_U_Y  = END_DEPTH_Y
    END_AB_U_Y    = BEGIN_AB_U_Y + (Parameters_RM_DEPTH_AHAT.WIDTH // 4)
    BEGIN_AB_V_Y  = END_AB_U_Y
    END_AB_V_Y    = BEGIN_AB_V_Y + (Parameters_RM_DEPTH_AHAT.WIDTH // 4)


def _unpack_rm_depth_ahat_nv12_as_yuv420p(yuv):
    y = yuv[_Mode0Layout_RM_DEPTH_AHAT.BEGIN_DEPTH_Y : _Mode0Layout_RM_DEPTH_AHAT.END_DEPTH_Y, :]
    u = yuv[_Mode0Layout_RM_DEPTH_AHAT.BEGIN_AB_U_Y  : _Mode0Layout_RM_DEPTH_AHAT.END_AB_U_Y,  :].reshape((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH // 4))
    v = yuv[_Mode0Layout_RM_DEPTH_AHAT.BEGIN_AB_V_Y  : _Mode0Layout_RM_DEPTH_AHAT.END_AB_V_Y,  :].reshape((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH // 4))

    ab = np.zeros((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH), dtype=np.uint8)

    ab[:, 0::4] = u
    ab[:, 1::4] = u
    ab[:, 2::4] = v
    ab[:, 3::4] = v

    return _RM_Depth_Frame(y, ab)


class decode_rm_depth_ahat:
    def __init__(self, profile):
        self.profile = profile
   
    def create(self):
        self._codec = av.CodecContext.create(get_video_codec_name(self.profile), 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return _unpack_rm_depth_ahat_nv12_as_yuv420p(frame.to_ndarray())
        return None


def decode_rm_depth_longthrow(payload):
    composite = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    h, w, _ = composite.shape
    image = composite.view(np.uint16).reshape((2*h, w))
    return _RM_Depth_Frame(image[:h, :], image[h:, :])


#------------------------------------------------------------------------------
# RM IMU Unpacker
#------------------------------------------------------------------------------

class _RM_IMU_Frame:
    def __init__(self, sensor_ticks_ns, x, y, z):
        self.sensor_ticks_ns = sensor_ticks_ns
        self.x               = x
        self.y               = y
        self.z               = z


class unpack_rm_imu:
    def __init__(self, payload):
        self._count = len(payload) // 28
        self._batch = payload

    def get_count(self):
        return self._count

    def get_frame(self, index):
        data = struct.unpack('<QQfff', self._batch[(index * 28):((index + 1) * 28)])
        return _RM_IMU_Frame(data[0], data[2], data[3], data[4])


#------------------------------------------------------------------------------
# PV Decoder
#------------------------------------------------------------------------------

class _PV_Frame:
    def __init__(self, image, focal_length, principal_point):
        self.image           = image
        self.focal_length    = np.frombuffer(focal_length, dtype=np.float32)
        self.principal_point = np.frombuffer(principal_point, dtype=np.float32)


def unpack_pv(payload):
    return _PV_Frame(payload[:-16], payload[-16:-8], payload[-8:])


def get_nv12_stride(width):
    return width + ((64 - (width & 63)) & 63)


class decode_pv:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = av.CodecContext.create(get_video_codec_name(self.profile), 'r')

    def decode(self, payload, format):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return frame.to_ndarray(format=format)
        return None


#------------------------------------------------------------------------------
# Microphone Decoder
#------------------------------------------------------------------------------

class decode_microphone:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = av.CodecContext.create(get_audio_codec_name(self.profile), 'r')

    def decode(self, payload):
        for packet in self._codec.parse(payload):
            for frame in self._codec.decode(packet):
                return frame.to_ndarray()
        return None


#------------------------------------------------------------------------------
# SI Unpacker
#------------------------------------------------------------------------------

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
    END_VALID           = BEGIN_VALID + 1
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
        self._valid = np.frombuffer(payload[_Mode0Layout_SI.BEGIN_VALID : _Mode0Layout_SI.END_VALID], dtype=np.uint8)

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
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_rm_vlc(rx_rm_vlc):
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        super().__init__(host, port, chunk, mode, profile, bitrate)
        self._codec = decode_rm_vlc(profile)

    def open(self):
        self._codec.create()
        super().open()
        self.get_next_packet()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_rm_depth_ahat(rx_rm_depth_ahat):
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        super().__init__(host, port, chunk, mode, profile, bitrate)
        self._codec = decode_rm_depth_ahat(profile)

    def open(self):
        self._codec.create()
        super().open()
        self.get_next_packet()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_rm_depth_longthrow(rx_rm_depth_longthrow):
    def __init__(self, host, port, chunk, mode, png_filter):
        super().__init__(host, port, chunk, mode, png_filter)

    def open(self):
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = decode_rm_depth_longthrow(data.payload)
        return data

    def close(self):
        super().close()


class rx_decoded_pv(rx_pv):
    def __init__(self, host, port, chunk, mode, width, height, framerate, profile, bitrate, format):
        super().__init__(host, port, chunk, mode, width, height, framerate, profile, bitrate)
        self.format = format
        self._codec = decode_pv(profile)

    def open(self):        
        self._codec.create()
        super().open()
        self.get_next_packet()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = unpack_pv(data.payload)
        data.payload.image = self._codec.decode(data.payload.image, self.format)
        return data

    def close(self):
        super().close()


class rx_decoded_microphone(rx_microphone):
    def __init__(self, host, port, chunk, profile):
        super().__init__(host, port, chunk, profile)
        self._codec = decode_microphone(profile)
        
    def open(self):
        self._codec.create()
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
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
    FLOAT_COUNT                = 2 + 2 + 3 + 2 + 16


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
    def __init__(self, focal_length, principal_point, radial_distortion, tangential_distortion, projection, intrinsics):
        self.focal_length          = focal_length
        self.principal_point       = principal_point
        self.radial_distortion     = radial_distortion
        self.tangential_distortion = tangential_distortion
        self.projection            = projection
        self.intrinsics            = intrinsics


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

    intrinsics = np.array([[-focal_length[0], 0, 0, 0], [0, focal_length[1], 0, 0], [principal_point[0], principal_point[1], 1, 0], [0, 0, 0, 1]], dtype=np.float32)

    return _Mode2_PV(focal_length, principal_point, radial_distortion, tangential_distortion, projection, intrinsics)


#------------------------------------------------------------------------------
# Port Information
#------------------------------------------------------------------------------

class _PortName:
    OF = ['rm_vlc_leftfront',
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
          'scene_understanding']


def get_port_index(port):
    return port - StreamPort.RM_VLC_LEFTFRONT


def get_port_name(port):
    return _PortName.OF[get_port_index(port)]


#------------------------------------------------------------------------------
# Remote Configuration
#------------------------------------------------------------------------------

class ipc_rc:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def _open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

    def _close(self):
        self._client.close()

    def set_hs_marker_state(self, state):
        self._open()
        command = struct.pack('<BI', 0x00, state)
        self._client.sendall(command)
        self._close()

    def set_pv_focus(self, focusmode, autofocusrange, distance, value, driverfallback):
        self._open()
        command = struct.pack('<BIIIII', 0x01, focusmode, autofocusrange, distance, value, driverfallback)
        self._client.sendall(command)
        self._close()

    def set_pv_video_temporal_denoising(self, mode):
        self._open()
        command = struct.pack('<BI', 0x02, mode)
        self._client.sendall(command)
        self._close()

    def set_pv_white_balance_preset(self, preset):
        self._open()
        command = struct.pack('<BI', 0x03, preset)
        self._client.sendall(command)
        self._close()

    def set_pv_white_balance_value(self, value):
        self._open()
        command = struct.pack('<BI', 0x04, value)
        self._client.sendall(command)
        self._close()

    def set_pv_exposure(self, mode, value):
        self._open()
        command = struct.pack('<BII', 0x05, mode, value)
        self._client.sendall(command)
        self._close()

    def get_application_version(self):
        self._open()
        command = struct.pack('<B', 0x06)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.SHORT * 4, ChunkSize.SINGLE_TRANSFER)
        version = struct.unpack('<HHHH', data)
        self._close()
        return version

    def get_utc_offset(self, samples):
        self._open()
        command = struct.pack('<BI', 0x07, samples)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.LONGLONG, ChunkSize.SINGLE_TRANSFER)
        self._close()
        return struct.unpack('<Q', data)[0]

    def set_pv_exposure_priority_video(self, enabled):
        self._open()
        command = struct.pack('<BI', 0x08, enabled)
        self._client.sendall(command)
        self._client.close()

    def set_pv_scene_mode(self, mode):
        self._open()
        command = struct.pack('<BI', 0x09, mode)
        self._client.sendall(command)
        self._client.close()

    def set_pv_iso_speed(self, mode, value):
        self._open()
        command = struct.pack('<BII', 0x0A, mode, value)
        self._client.sendall(command)
        self._client.close()

    def get_pv_subsystem_status(self):
        self._open()
        command = struct.pack('<B', 0x0B)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.BYTE, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<B', data)[0] != 0

    def wait_for_pv_subsystem(self, status):
        while (self.get_pv_subsystem_status() != status):
            pass


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

    def add_box(self, center, extents):
        self._data.extend(struct.pack('<Iffffff', _SM_VolumeType.Box, center[0], center[1], center[2], extents[0], extents[1], extents[2]))
        self._count += 1

    def add_frustum(self, near, far, right, left, top, bottom):
        self._data.extend(struct.pack('<Iffffffffffffffffffffffff', _SM_VolumeType.Frustum, near[0], near[1], near[2], near[3], far[0], far[1], far[2], far[3], right[0], right[1], right[2], right[3], left[0], left[1], left[2], left[3], top[0], top[1], top[2], top[3], bottom[0], bottom[1], bottom[2], bottom[3]))
        self._count += 1

    def add_oriented_box(self, center, extents, orientation):
        self._data.extend(struct.pack('<Iffffffffff', _SM_VolumeType.OrientedBox, center[0], center[1], center[2], extents[0], extents[1], extents[2], orientation[0], orientation[1], orientation[2], orientation[3]))
        self._count += 1

    def add_sphere(self, center, radius):
        self._data.extend(struct.pack('<Iffff', _SM_VolumeType.Sphere, center[0], center[1], center[2], radius))
        self._count += 1

    def _get(self):
        return self._count, self._data


class sm_mesh_task:
    def __init__(self):
        self._count = 0
        self._data = bytearray()

    def add_task(self, id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format, include_vertex_normals):
        self._data.extend(struct.pack('<16sdIIII', id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format, 1 if include_vertex_normals else 0))
        self._count += 1

    def _get(self):
        return self._count, self._data


class _sm_mesh:
    def __init__(self, vertex_position_scale, update_time, pose, bounds, vertex_positions, triangle_indices, vertex_normals):
        self.vertex_position_scale = vertex_position_scale
        self.update_time           = update_time
        self.pose                  = pose
        self.bounds                = bounds        
        self.vertex_positions      = vertex_positions
        self.triangle_indices      = triangle_indices
        self.vertex_normals        = vertex_normals

    def unpack(self, vertex_position_format, triangle_index_format, vertex_normal_format):
        self.vertex_position_scale = np.frombuffer(self.vertex_position_scale, dtype=np.float32).reshape((1, 3))
        self.update_time           = np.frombuffer(self.update_time,           dtype=np.uint64)
        self.pose                  = np.frombuffer(self.pose,                  dtype=np.float32).reshape((4, 4))
        self.bounds                = np.frombuffer(self.bounds,                dtype=np.float32)        
        self.vertex_positions      = np.frombuffer(self.vertex_positions,      dtype=_SM_Convert.DirectXPixelFormatToNumPy[vertex_position_format]).reshape((-1, 4))
        self.triangle_indices      = np.frombuffer(self.triangle_indices,      dtype=_SM_Convert.DirectXPixelFormatToNumPy[triangle_index_format]).reshape((-1, 3))
        self.vertex_normals        = np.frombuffer(self.vertex_normals,        dtype=_SM_Convert.DirectXPixelFormatToNumPy[vertex_normal_format]).reshape((-1, 4))


class ipc_sm:
    _CMD_CREATE_OBSERVER       = 0x00
    _CMD_SET_VOLUMES           = 0x01
    _CMD_GET_OBSERVED_SURFACES = 0x02
    _CMD_GET_MESHES            = 0x03

    _MESH_INFO_HEADER_SIZE     = 144

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self._client = _client()
        
    def open(self):
        self._client.open(self.host, self.port)

    def create_observer(self):
        self._client.sendall(struct.pack('<B', ipc_sm._CMD_CREATE_OBSERVER))

    def set_volumes(self, volumes):
        count, data = volumes._get()
        msg = bytearray()
        msg.extend(struct.pack('<BB', ipc_sm._CMD_SET_VOLUMES, count))
        msg.extend(data)
        self._client.sendall(msg)

    def get_observed_surfaces(self):
        self._client.sendall(struct.pack('<B', ipc_sm._CMD_GET_OBSERVED_SURFACES))
        count = struct.unpack('<Q', self._client.download(_SIZEOF.QWORD, ChunkSize.SINGLE_TRANSFER))[0]
        ids = self._client.download(count * 16, ChunkSize.SINGLE_TRANSFER)
        return [ids[(i*16):((i+1)*16)] for i in range(0, count)]
    
    def _download_mesh(self):
        header = self._client.download(ipc_sm._MESH_INFO_HEADER_SIZE, ChunkSize.SINGLE_TRANSFER)
        index, status, vpl, til, vnl = struct.unpack('<IIIII', header[:20])

        if (status != 0):
            return index, None
        
        payload = self._client.download(vpl + til + vnl, ChunkSize.SINGLE_TRANSFER)

        scale       = header[20:32]
        update_time = header[32:40]
        pose        = header[40:104]
        bounds      = header[104:144]

        vpd_b = 0
        vpd_e = vpd_b + vpl
        tid_b = vpd_e
        tid_e = tid_b + til
        vnd_b = tid_e
        vnd_e = vnd_b + vnl
        
        vpd = payload[vpd_b:vpd_e]
        tid = payload[tid_b:tid_e]
        vnd = payload[vnd_b:vnd_e]

        return index, _sm_mesh(scale, update_time, pose, bounds, vpd, tid, vnd)
    
    def _download_meshes(self, count):
        for _ in range(0, count):
            yield self._download_mesh()
    
    def get_meshes(self, tasks, threads):
        count, data = tasks._get()
        msg = bytearray()
        msg.extend(struct.pack('<BII', ipc_sm._CMD_GET_MESHES, count, threads))
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
    Background = 0,
    Wall = 1,
    Floor = 2,
    Ceiling = 3,
    Platform = 4,
    Unknown = 247,
    World = 248,
    CompletelyInferred = 249,


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
        self.alignment = np.frombuffer(self.alignment, np.int32)
        self.extents = np.frombuffer(self.extents, dtype=np.float32)


class _su_result:
    def __init__(self, extrinsics, pose, items):        
        self.extrinsics = extrinsics
        self.pose = pose
        self.items = items

    def unpack(self):
        self.extrinsics = np.frombuffer(self.extrinsics, dtype=np.float32).reshape((4, 4))
        self.pose = np.frombuffer(self.pose, dtype=np.float32).reshape((4, 4))


class ipc_su:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self._client = _client()

    def open(self):
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
        msg = bytearray()
        msg.extend(struct.pack('<BBBBIfBBBBBBBBI', task.enable_quads, task.enable_meshes, task.enable_only_observed, task.enable_world_mesh, task.mesh_lod, task.query_radius, task.create_mode, task.kind_flags, task.get_orientation, task.get_position, task.get_location_matrix, task.get_quad, task.get_meshes, task.get_collider_meshes, len(task.guid_list)))
        for guid in task.guid_list:
            msg.extend(guid)
        self._client.sendall(msg)
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


#//////////////////////////////////////////////////////////////////////////////
# Extension: redis-streamer (NYU)
#//////////////////////////////////////////////////////////////////////////////

#------------------------------------------------------------------------------
# GOP Tagging
#------------------------------------------------------------------------------

class _extension_gop:
    def __init__(self, gop_size):
        self.aliased_index = 0
        self.gop_size = gop_size

    def extend(self, data):
        data.extend(struct.pack('<B', self.aliased_index))
        self.aliased_index = (self.aliased_index + 1) % self.gop_size


#------------------------------------------------------------------------------
# API redis-streamer
#------------------------------------------------------------------------------

def is_rs_host(host):
    return ':' in host


def _rs_get_stream_url_push(host, port):
    return f'ws://{host}/data/{get_port_name(port)}/push?header=0'


def _rs_get_stream_url_pull(host, port):
    return f'ws://{host}/data/{get_port_name(port)}/pull?header=0'


#------------------------------------------------------------------------------
# Network Client (Websockets)
#------------------------------------------------------------------------------

class _rs_client:
    def open(self, host, port, max_size):
        self._loop = asyncio.get_event_loop()
        self._client = self._loop.run_until_complete(websockets.client.connect(_rs_get_stream_url_pull(host, port), max_size=max_size, compression=None))

    def recv(self):
        while (True):
            data = self._loop.run_until_complete(self._client.recv())
            if (len(data) > 0):
                return data

    def close(self):
        self._loop.run_until_complete(self._client.close())


#------------------------------------------------------------------------------
# Packet Gatherer (Websockets)
#------------------------------------------------------------------------------

class _rs_gatherer:
    def open(self, host, port, max_size):
        self._genlock = False
        self._client = _rs_client()
        self._client.open(host, port, max_size)

    def _fetch(self):
        data = self._client.recv()
        raw_packet = data[:-1]
        aliased_index = struct.unpack('<B', data[-1:])[0]
        return (aliased_index, raw_packet)
    
    def get_next_packet(self):
        aliased_index, data = self._fetch()
        while (not self._genlock):
            if (aliased_index == 0): 
                self._genlock = True
            else:
                aliased_index, data = self._fetch()
        return unpack_packet(data)
    
    def close(self):
        self._client.close()

