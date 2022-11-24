
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


# IPC TCP Ports
class IPCPort:
    REMOTE_CONFIGURATION = 3809


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

        if (chunk_size > total):
            chunk_size = total

        while (total > 0):
            chunk = self.recv(chunk_size)
            data.extend(chunk)
            total -= len(chunk)
            if (chunk_size > total):
                chunk_size = total

        if (total != 0):
            raise Exception('download failed')

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


class _unpacker:
    def __init__(self, mode):
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


def pack_packet(packet):
    buffer = bytearray()
    buffer.extend(struct.pack('<QI', packet.timestamp, len(packet.payload)))
    buffer.extend(packet.payload)
    if (packet.pose is not None):
        buffer.extend(packet.pose.tobytes())
    return buffer


def is_valid_pose(pose):
    return pose[3, 3] != 0


#------------------------------------------------------------------------------
# Packet Gatherer
#------------------------------------------------------------------------------

class _gatherer:
    def open(self, host, port, chunk_size, mode):
        self._client = _client()
        self._unpacker = _unpacker(mode)
        self._chunk_size = chunk_size

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
# File I/O
#------------------------------------------------------------------------------

class _Header:
    def __init__(self, header):
        self.port = header[0]
        self.mode = header[1]
        self.profile = header[2]


class writer:
    def open(self, filename, port, mode, profile):
        self._data = open(filename, 'wb')
        self._data.write(struct.pack('<HBB', port, mode, profile if (profile is not None) else _RANGEOF.U8_MAX))

    def write(self, packet):
        self._data.write(pack_packet(packet))

    def close(self):
        self._data.close()


class reader:
    def open(self, filename, chunk_size):
        self._data = open(filename, 'rb')        
        self._header = _Header(struct.unpack('<HBB', self._data.read(_SIZEOF.WORD + 2 * _SIZEOF.BYTE)))
        self._unpacker = _unpacker(self._header.mode)
        self._chunk_size = chunk_size
        self._eof = False

    def get_header(self):
        return self._header
        
    def read(self):
        while (True):
            if (self._unpacker.unpack()):
                return self._unpacker.get()
            if (self._eof):
                return None

            chunk = self._data.read(self._chunk_size)
            self._eof = len(chunk) < self._chunk_size
            self._unpacker.extend(chunk)

    def close(self):
        self._data.close()


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
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_vlc(mode, profile, bitrate))
    return c


def _connect_client_rm_depth_ahat(host, port, chunk_size, mode, profile, bitrate):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_depth_ahat(mode, profile, bitrate))
    return c


def _connect_client_rm_depth_longthrow(host, port, chunk_size, mode, png_filter):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_depth_longthrow(mode, png_filter))
    return c


def _connect_client_rm_imu(host, port, chunk_size, mode):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_rm_imu(mode))
    return c


def _connect_client_pv(host, port, chunk_size, mode, width, height, framerate, profile, bitrate):
    c = _gatherer()
    c.open(host, port, chunk_size, mode)
    c.sendall(_create_configuration_for_pv(mode, width, height, framerate, profile, bitrate))
    return c


def _connect_client_microphone(host, port, chunk_size, profile):
    c = _gatherer()
    c.open(host, port, chunk_size, StreamMode.MODE_0)
    c.sendall(_create_configuration_for_microphone(profile))
    return c


def _connect_client_si(host, port, chunk_size):
    c = _gatherer()
    c.open(host, port, chunk_size, StreamMode.MODE_0)
    return c


def start_subsystem_pv(host, port):
    c = _client()
    c.open(host, port)
    c.sendall(_create_configuration_for_pv_mode2(0x7, 1920, 1080, 30))
    c.close()


def stop_subsystem_pv(host, port):
    c = _client()
    c.open(host, port)
    c.sendall(_create_configuration_for_pv_mode2(0xB, 1920, 1080, 30))
    c.close()


#------------------------------------------------------------------------------
# Receiver Wrappers
#------------------------------------------------------------------------------

class rx_rm_vlc:
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
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class rx_rm_depth_ahat:
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


class rx_rm_depth_longthrow:
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


class rx_rm_imu:
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


class rx_pv:
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


class rx_microphone:
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


class rx_si:
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

class rx_decoded_rm_vlc:
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        self._client = rx_rm_vlc(host, port, chunk, mode, profile, bitrate)
        self._codec = decode_rm_vlc(profile)

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


class rx_decoded_rm_depth_ahat:
    def __init__(self, host, port, chunk, mode, profile, bitrate):
        self._client = rx_rm_depth_ahat(host, port, chunk, mode, profile, bitrate)
        self._codec = decode_rm_depth_ahat(profile)

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


class rx_decoded_rm_depth_longthrow:
    def __init__(self, host, port, chunk, mode, png_filter):
        self._client = rx_rm_depth_longthrow(host, port, chunk, mode, png_filter)

    def open(self):
        self._client.open()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        data.payload = decode_rm_depth_longthrow(data.payload)
        return data

    def close(self):
        self._client.close()


class rx_decoded_pv:
    def __init__(self, host, port, chunk, mode, width, height, framerate, profile, bitrate, format):
        self._client = rx_pv(host, port, chunk, mode, width, height, framerate, profile, bitrate)
        self._codec = decode_pv(profile)
        self._format = format

    def open(self):        
        self._codec.create()
        self._client.open()
        self.get_next_packet()

    def get_next_packet(self):
        data = self._client.get_next_packet()
        data.payload = self._codec.decode(data.payload, self._format)
        return data

    def close(self):
        self._client.close()


class rx_decoded_microphone:
    def __init__(self, host, port, chunk, profile):
        self._client = rx_microphone(host, port, chunk, profile)
        self._codec = decode_microphone(profile)
        
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
          'spatial_input']


def get_port_index(port):
    return port - StreamPort.RM_VLC_LEFTFRONT


def get_port_name(port):
    return _PortName.OF[get_port_index(port)]


#------------------------------------------------------------------------------
# Remote Configuration
#------------------------------------------------------------------------------

class tx_rc:
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

