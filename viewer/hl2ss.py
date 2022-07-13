
import numpy as np
import socket
import struct
import time
import cv2

# Stream Ports
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

# RM VLC Resolution
class Resolution_RM_VLC:
    WIDTH  = 640
    HEIGHT = 480
    FPS    = 30
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)

# RM Depth Long Throw Resolution
class Resolution_RM_DEPTH_LONGTHROW:
    WIDTH  = 320
    HEIGHT = 288
    FPS    = 5
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)

# Time base for all timestamps
class TimeBase:
    HUNDREDS_OF_NANOSECONDS = 10*1000*1000










class _SIZEOF:
    FLOAT = 4



#------------------------------------------------------------------------------
# Network Client
#------------------------------------------------------------------------------

class client:
    DEFAULT_CHUNK_SIZE = 4096

    def __init__(self):
        self._socket = None

    def open(self, host, port):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((host, port))

    def sendall(self, data):
        self._socket.sendall(data)

    def recv(self, chunk_size):
        chunk = self._socket.recv(chunk_size)
        if (len(chunk) <= 0):
            raise('connection closed')
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
            raise('download failed')

        return data

    def close(self):
        self._socket.close()







class unpacker:
    def __init__(self, mode):
        self._mode = mode
        self._state = 0
        self._buffer = bytearray()
        self._timestamp = None
        self._size = None
        self._payload = None
        self._pose = None

    def unpack(self, chunk):
        self._buffer.extend(chunk)
        length = len(self._buffer)
        
        while True:
            if (self._state == 0):
                if (length >= 12):
                    header = struct.unpack('<QI', self._buffer[:12])
                    self._timestamp = header[0]
                    self._size = 12 + header[1]
                    if (self._mode == 1):
                        self._size += 64
                    self._state = 1
                    continue
            elif (self._state == 1):
                if (length >= self._size):
                    if (self._mode == 1):
                        payload_end = self._size - 64
                    else:
                        payload_end = self._size
                    self._payload = self._buffer[12:payload_end]
                    self._pose = self._buffer[payload_end:self._size]
                    self._buffer = self._buffer[self._size:]
                    self._state = 0
                    return True
            return False

    def get_timestamp(self):
        return self._timestamp

    def get_payload(self):
        return self._payload

    def get_pose(self):
        if (self._mode == StreamMode.MODE_1):
            return np.frombuffer(self._pose, dtype=np.float32).reshape((4, 4))
        else:
            return None




class packet:
    def __init__(self, timestamp, payload, pose):
        self.timestamp = timestamp
        self.payload   = payload
        self.pose      = pose


class gatherer:
    def open(self, host, port, chunk_size, mode):
        self._client     = client()
        self._unpacker   = unpacker(mode)
        self._chunk_size = chunk_size

        self._client.open(host, port)
        
    def configure(self, configuration):
        self._client.sendall(configuration)

    def get_next_packet(self):
        while True:
            if (self._unpacker.unpack(self._client.recv(self._chunk_size))):
                return packet(self._unpacker.get_timestamp(), self._unpacker.get_payload(), self._unpacker.get_pose())

    def close(self):
        self._client.close()






def unpack_rm_depth(payload):
    composite = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    h, w, _ = composite.shape
    interleaved = composite.view(np.uint16).reshape((h, w, 2))
    depth, ab = np.dsplit(interleaved, 2)
    return (depth, ab)


class unpacker_rm_imu:
    def __init__(self, payload):
        self._count = int(len(payload) / 28)
        self._batch = payload

    def get_count(self):
        return self._count

    def get_sample(self, index):
        return struct.unpack('<QQfff', self._batch[(index * 28):((index + 1) * 28)])



#------------------------------------------------------------------------------
# Codecs
#------------------------------------------------------------------------------

def get_video_codec_name(profile):
    if (profile == VideoProfile.H265_MAIN):
        return 'hevc'
    else:
        return 'h264'


def get_audio_codec_name(profile):
    return 'aac'


#------------------------------------------------------------------------------
# Stream Configuration
#------------------------------------------------------------------------------

def create_configuration_for_mode(mode):
    return struct.pack('<B', mode)


def create_configuration_for_video(mode, width, height, framerate, profile, bitrate):
    return struct.pack('<BHHBBI', mode, width, height, framerate, profile, bitrate)


def create_configuration_for_audio(profile):
    return struct.pack('<B', profile)


#------------------------------------------------------------------------------
# Mode 2 Data Acquisition
#------------------------------------------------------------------------------

class _Mode2Layout_RM_VLC:
    BEGIN_UV2X       = 0
    END_UV2X         = BEGIN_UV2X + Resolution_RM_VLC.PIXELS
    BEGIN_UV2Y       = END_UV2X
    END_UV2Y         = BEGIN_UV2Y + Resolution_RM_VLC.PIXELS
    BEGIN_EXTRINSICS = END_UV2Y
    END_EXTRINSICS   = BEGIN_EXTRINSICS + 16
    FLOAT_COUNT      = 2*Resolution_RM_VLC.PIXELS + 16


class _Mode2Layout_RM_DEPTH_LONGTHROW:
    BEGIN_UV2X       = 0
    END_UV2X         = BEGIN_UV2X + Resolution_RM_DEPTH_LONGTHROW.PIXELS
    BEGIN_UV2Y       = END_UV2X
    END_UV2Y         = BEGIN_UV2Y + Resolution_RM_DEPTH_LONGTHROW.PIXELS
    BEGIN_EXTRINSICS = END_UV2Y
    END_EXTRINSICS   = BEGIN_EXTRINSICS + 16
    BEGIN_SCALE      = END_EXTRINSICS
    END_SCALE        = BEGIN_SCALE + 1
    FLOAT_COUNT      = 2*Resolution_RM_DEPTH_LONGTHROW.PIXELS + 16 + 1


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


class Mode2_RM_VLC:
    def __init__(self, uv2xy, extrinsics):
        self.uv2xy      = uv2xy
        self.extrinsics = extrinsics


class Mode2_RM_DEPTH:
    def __init__(self, uv2xy, extrinsics, scale):
        self.uv2xy      = uv2xy
        self.extrinsics = extrinsics
        self.scale      = scale


class Mode2_RM_IMU:
    def __init__(self, extrinsics):
        self.extrinsics = extrinsics


class Mode2_PV:
    def __init__(self, focal_length, principal_point, radial_distortion, tangential_distortion, projection):
        self.focal_length          = focal_length
        self.principal_point       = principal_point
        self.radial_distortion     = radial_distortion
        self.tangential_distortion = tangential_distortion
        self.projection            = projection


def _download_mode2_data(host, port, configuration, bytes):
    c = client()

    c.open(host, port)
    c.sendall(configuration)
    data = c.download(bytes, client.DEFAULT_CHUNK_SIZE)
    c.close()

    return data


def get_mode2_rm_vlc(host, port):
    data   = _download_mode2_data(host, port, create_configuration_for_mode(StreamMode.MODE_2), _Mode2Layout_RM_VLC.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    uv2x       = floats[_Mode2Layout_RM_VLC.BEGIN_UV2X       : _Mode2Layout_RM_VLC.END_UV2X].reshape(Resolution_RM_VLC.SHAPE)
    uv2y       = floats[_Mode2Layout_RM_VLC.BEGIN_UV2Y       : _Mode2Layout_RM_VLC.END_UV2Y].reshape(Resolution_RM_VLC.SHAPE)
    extrinsics = floats[_Mode2Layout_RM_VLC.BEGIN_EXTRINSICS : _Mode2Layout_RM_VLC.END_EXTRINSICS].reshape((4, 4))

    return Mode2_RM_VLC(np.dstack((uv2x, uv2y)), extrinsics)


def get_mode2_rm_depth(host, port):
    if (port == StreamPort.RM_DEPTH_AHAT):
        return None

    data   = _download_mode2_data(host, port, create_configuration_for_mode(StreamMode.MODE_2), _Mode2Layout_RM_DEPTH_LONGTHROW.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    uv2x       = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_UV2X       : _Mode2Layout_RM_DEPTH_LONGTHROW.END_UV2X].reshape(Resolution_RM_DEPTH_LONGTHROW.SHAPE)
    uv2y       = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_UV2Y       : _Mode2Layout_RM_DEPTH_LONGTHROW.END_UV2Y].reshape(Resolution_RM_DEPTH_LONGTHROW.SHAPE)
    extrinsics = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_EXTRINSICS : _Mode2Layout_RM_DEPTH_LONGTHROW.END_EXTRINSICS].reshape((4, 4))
    scale      = floats[_Mode2Layout_RM_DEPTH_LONGTHROW.BEGIN_SCALE      : _Mode2Layout_RM_DEPTH_LONGTHROW.END_SCALE]

    return Mode2_RM_DEPTH(np.dstack((uv2x, uv2y)), extrinsics, scale)


def get_mode2_rm_imu(host, port):
    if (port == StreamPort.RM_IMU_MAGNETOMETER):
        return None

    data   = _download_mode2_data(host, port, create_configuration_for_mode(StreamMode.MODE_2), _Mode2Layout_RM_IMU.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    extrinsics = floats[_Mode2Layout_RM_IMU.BEGIN_EXTRINSICS : _Mode2Layout_RM_IMU.END_EXTRINSICS].reshape((4, 4))

    return Mode2_RM_IMU(extrinsics)


def get_mode2_pv(host, port, width, height, framerate, profile, bitrate):
    data   = _download_mode2_data(host, port, create_configuration_for_video(StreamMode.MODE_2, width, height, framerate, profile, bitrate), _Mode2Layout_PV.FLOAT_COUNT * _SIZEOF.FLOAT)
    floats = np.frombuffer(data, dtype=np.float32)

    focal_length          = floats[_Mode2Layout_PV.BEGIN_FOCALLENGTH          : _Mode2Layout_PV.END_FOCALLENGTH]
    principal_point       = floats[_Mode2Layout_PV.BEGIN_PRINCIPALPOINT       : _Mode2Layout_PV.END_PRINCIPAL_POINT]
    radial_distortion     = floats[_Mode2Layout_PV.BEGIN_RADIALDISTORTION     : _Mode2Layout_PV.END_RADIALDISTORTION]
    tangential_distortion = floats[_Mode2Layout_PV.BEGIN_TANGENTIALDISTORTION : _Mode2Layout_PV.END_TANGENTIALDISTORTION]
    projection            = floats[_Mode2Layout_PV.BEGIN_PROJECTION           : _Mode2Layout_PV.END_PROJECTION].reshape((4, 4))

    return Mode2_PV(focal_length, principal_point, radial_distortion, tangential_distortion, projection)


#------------------------------------------------------------------------------
# Utilities
#------------------------------------------------------------------------------

class counter:
    def __init__(self):
        self.reset()

    def increment(self, count=1):
        self._count += count
        return self._count

    def value(self):
        return self._count

    def reset(self):
        self._count = 0


class framerate_counter:
    def __init__(self):
        self._frames = counter()
        self._start = None

    def increment(self, count=1):
        if (self._start is None):
            self._start = time.perf_counter()
        return self._frames.increment(count)

    def value(self):
        return self._frames.value()

    def reset(self):
        self._frames.reset()
        self._start = None

    def pop(self):
        ts = time.perf_counter()
        value = self._frames.value() / (ts - self._start)
        self._frames.reset()
        self._start = ts
        return value


class continuity_analyzer:
    def __init__(self, period):
        self._last = None
        self._period = period

    def check(self, timestamp):
        if (self._last is not None):
            delta = timestamp - self._last
            drop = delta > (1.75 * self._period)
        else:
            drop = False
        self._last = timestamp
        return drop
