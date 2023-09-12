
import asyncio
import cv2
import numpy as np
import websockets.client
import BBN_redis_frame_load as holoframe
import hl2ss


#//////////////////////////////////////////////////////////////////////////////
# Extension: redis-streamer (NYU)
#//////////////////////////////////////////////////////////////////////////////

REDIS_STREAMER_PORT = '8000'


#------------------------------------------------------------------------------
# Network Client (Websockets)
#------------------------------------------------------------------------------

class _rs_stream_name:
    OF = {
        hl2ss.StreamPort.RM_VLC_LEFTLEFT      : "gll",
        hl2ss.StreamPort.RM_VLC_LEFTFRONT     : "glf",
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : "grf",
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : "grr",
        hl2ss.StreamPort.RM_DEPTH_LONGTHROW   : "depthlt",
        hl2ss.StreamPort.RM_IMU_ACCELEROMETER : "imuaccel",
        hl2ss.StreamPort.RM_IMU_GYROSCOPE     : "imugyro",
        hl2ss.StreamPort.RM_IMU_MAGNETOMETER  : "imumag",
        hl2ss.StreamPort.PERSONAL_VIDEO       : "main",
        hl2ss.StreamPort.MICROPHONE           : "mic0",
        hl2ss.StreamPort.SPATIAL_INPUT        : "si",
        # EET
    }


def is_rs_host(host):
    return ":" in host


def _rs_get_stream_url_push(host, port):
    return f"ws://{host}/data/{_rs_stream_name.OF[port]}/push?header=0"


def _rs_get_stream_url_pull(host, port):
    return f"ws://{host}/data/{_rs_stream_name.OF[port]}/pull?header=0&latest=1"


class _rs_client:
    def open(self, host, port, max_size):
        try:
            self._loop = asyncio.get_event_loop()
        except:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
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
        return self._client.recv()

    def get_next_packet(self):
        data = self._fetch()
        d = holoframe.load(data)
        ft = d["frame_type"]

        if ft == holoframe.SensorType.PV:
            return hl2ss._packet(d["time"], hl2ss._PV_Frame(cv2.cvtColor(d["image"][:, :, ::-1], cv2.COLOR_RGB2BGR), [d["focalX"], d["focalY"]], [d["principalX"], d["principalY"]]), d["cam2world"].transpose())
        if ft == holoframe.SensorType.GLF:
            return hl2ss._packet(d["time"], cv2.rotate(d["image"], cv2.ROTATE_90_COUNTERCLOCKWISE), d["rig2world"].transpose())
        if ft == holoframe.SensorType.GLL:
            return hl2ss._packet(d["time"], cv2.rotate(d["image"], cv2.ROTATE_90_CLOCKWISE), d["rig2world"].transpose())
        if ft == holoframe.SensorType.GRF:
            return hl2ss._packet(d["time"], cv2.rotate(d["image"], cv2.ROTATE_90_CLOCKWISE), d["rig2world"].transpose())
        if ft == holoframe.SensorType.GRR:
            return hl2ss._packet(d["time"], cv2.rotate(d["image"], cv2.ROTATE_90_COUNTERCLOCKWISE), d["rig2world"].transpose())
        if ft == holoframe.SensorType.DepthLT:
            return hl2ss._packet(d["time"], hl2ss._RM_Depth_Frame(d["image"], d["infrared"].astype(np.uint16)), d["rig2world"].transpose())
        if ft == holoframe.SensorType.Microphone:
            return hl2ss._packet(d["time"], d["data"], None)
        if ft == holoframe.SensorType.SpatialInput:
            return hl2ss._packet(d["time"], d["data"], None)

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Acquisition
#------------------------------------------------------------------------------

def _connect_client(host, port):
    c = _rs_gatherer()
    c.open(host, port, None)
    return c


def start_subsystem_pv(host, port):
    pass


def stop_subsystem_pv(host, port):
    pass


#------------------------------------------------------------------------------
# Receiver Wrappers
#------------------------------------------------------------------------------

class _rx(hl2ss._context_manager):
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def open(self):
        self._client = _connect_client(self.host + ':' + REDIS_STREAMER_PORT, self.port)
        
    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


class _rx_rm_vlc(_rx):
    def __init__(self, host, port, chunk, mode, divisor, profile, level, bitrate, options):
        super().__init__(host, port)
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile = profile
        self.level = level
        self.bitrate = bitrate
        self.options = options


class _rx_rm_depth_ahat(_rx):
    def __init__(self, host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options):
        super().__init__(host, port)
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.profile_z = profile_z
        self.profile_ab = profile_ab
        self.level = level
        self.bitrate = bitrate
        self.options = options


class _rx_rm_depth_longthrow(_rx):
    def __init__(self, host, port, chunk, mode, divisor, png_filter):
        super().__init__(host, port)
        self.chunk = chunk
        self.mode = mode
        self.divisor = divisor
        self.png_filter = png_filter


class _rx_rm_imu(_rx):
    def __init__(self, host, port, chunk, mode):
        super().__init__(host, port)
        self.chunk = chunk
        self.mode = mode


class _rx_pv(_rx):
    def __init__(self, host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options):
        super().__init__(host, port)
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


class _rx_microphone(_rx):
    def __init__(self, host, port, chunk, profile, level):
        super().__init__(host, port)
        self.chunk = chunk
        self.profile = profile
        self.level = level


class _rx_si(_rx):
    def __init__(self, host, port, chunk):
        super().__init__(host, port)
        self.chunk = chunk


class _rx_eet(_rx):
    def __init__(self, host, port, chunk, fps):
        super().__init__(host, port)
        self.chunk = chunk
        self.fps = fps


#------------------------------------------------------------------------------
# Receivers
#------------------------------------------------------------------------------

def rx_rm_vlc(host, port, chunk=hl2ss.ChunkSize.RM_VLC, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile=hl2ss.VideoProfile.H264_HIGH, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    return _rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options)


def rx_rm_depth_ahat(host, port, chunk=hl2ss.ChunkSize.RM_DEPTH_AHAT, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile_z=hl2ss.DepthProfile.SAME, profile_ab=hl2ss.VideoProfile.H264_HIGH, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    return _rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)


def rx_rm_depth_longthrow(host, port, chunk=hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode=hl2ss.StreamMode.MODE_1, divisor=1, png_filter=hl2ss.PNGFilterMode.PAETH, decoded=True):
    return _rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter)


def rx_rm_imu(host, port, chunk=hl2ss.ChunkSize.RM_IMU, mode=hl2ss.StreamMode.MODE_1):
    return _rx_rm_imu(host, port, chunk, mode)


def rx_pv(host, port, chunk=hl2ss.ChunkSize.PERSONAL_VIDEO, mode=hl2ss.StreamMode.MODE_1, width=1920, height=1080, framerate=30, divisor=1, profile=hl2ss.VideoProfile.H264_HIGH, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded_format='bgr24'):
    return _rx_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)


def rx_microphone(host, port, chunk=hl2ss.ChunkSize.MICROPHONE, profile=hl2ss.AudioProfile.AAC_24000, level=hl2ss.AACLevel.L2, decoded=True):
    return _rx_microphone(host, port, chunk, profile, level)


def rx_si(host, port, chunk=hl2ss.ChunkSize.SPATIAL_INPUT):
    return _rx_si(host, port, chunk)


def rx_eet(host, port, chunk=hl2ss.ChunkSize.EXTENDED_EYE_TRACKER, fps=30):
    return _rx_eet(host, port, chunk, fps)

