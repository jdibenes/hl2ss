
from enum import IntEnum
from typing import List
import numpy as np
import quaternion

import time
import struct
import asyncio
import websockets.client
import cv2
import av
import logging

import zenoh
from zenoh import config, QueryTarget
from zenoh import Reliability

import hl2ss_schema
import hl2ss_core

log = logging.getLogger(__name__)


# Stream Types
class StreamType(IntEnum):
    RM_VLC_LEFTFRONT     = 0
    RM_VLC_LEFTLEFT      = 1
    RM_VLC_RIGHTFRONT    = 2
    RM_VLC_RIGHTRIGHT    = 3
    RM_DEPTH_AHAT        = 4
    RM_DEPTH_LONGTHROW   = 5
    RM_IMU_ACCELEROMETER = 6
    RM_IMU_GYROSCOPE     = 7
    RM_IMU_MAGNETOMETER  = 8
    PERSONAL_VIDEO       = 10
    MICROPHONE           = 11
    SPATIAL_INPUT        = 12
    EXTENDED_EYE_TRACKER = 17


# RPC Type
class RpcType(IntEnum):
    REMOTE_CONFIGURATION = 0
    SPATIAL_MAPPING      = 1
    SCENE_UNDERSTANDING  = 2
    VOICE_INPUT          = 3
    UNITY_MESSAGE_QUEUE  = 4


# Video Encoder Configuration
# 0: H264 base
# 1: H264 main
# 2: H264 high
# 3: H265 main (HEVC)
class VideoProfile(IntEnum):
    H264_BASE = 0
    H264_MAIN = 1
    H264_HIGH = 2
    H265_MAIN = 3
    RAW       = 4


map_videoprofile = {
    hl2ss_schema.Hololens2H26xProfile.H264Profile_Base: VideoProfile.H264_BASE,
    hl2ss_schema.Hololens2H26xProfile.H264Profile_Main: VideoProfile.H264_MAIN,
    hl2ss_schema.Hololens2H26xProfile.H264Profile_High: VideoProfile.H264_HIGH,
    hl2ss_schema.Hololens2H26xProfile.H265Profile_Main: VideoProfile.H265_MAIN,
    hl2ss_schema.Hololens2H26xProfile.H26xProfile_None: VideoProfile.RAW,
}

# Audio Encoder Configuration
# 0: AAC 12000 bytes/s
# 1: AAC 16000 bytes/s
# 2: AAC 20000 bytes/s
# 3: AAC 24000 bytes/s
class AudioProfile(IntEnum):
    AAC_12000 = 0
    AAC_16000 = 1
    AAC_20000 = 2
    AAC_24000 = 3
    RAW       = 4

map_audioprofile = {
    hl2ss_schema.Hololens2AACProfile.AACProfile_12000: AudioProfile.AAC_12000,
    hl2ss_schema.Hololens2AACProfile.AACProfile_16000: AudioProfile.AAC_16000,
    hl2ss_schema.Hololens2AACProfile.AACProfile_20000: AudioProfile.AAC_20000,
    hl2ss_schema.Hololens2AACProfile.AACProfile_24000: AudioProfile.AAC_24000,
    hl2ss_schema.Hololens2AACProfile.AACProfile_None: AudioProfile.RAW,
}

# PNG Filters
class PngFilterMode(IntEnum):
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


#------------------------------------------------------------------------------
# Pose
#------------------------------------------------------------------------------

def to_vec3(v):
    if isinstance(v, hl2ss_schema.Vector3):
        v = np.asarray([v.x, v.y, v.z])
    # more ..
    return v


def to_quat(v):
    if isinstance(v, hl2ss_schema.Quaternion):
        v = np.quaternion(v.w, v.x, v.y, v.z)
    # more ..
    return v


class Pose:
    def __init__(self, translation, orientation):
        self.translation = to_vec3(translation)
        self.orientation = to_quat(orientation)

    def to_matrix(self):
        mat = np.identity(4)
        mat[:3, :3] = quaternion.as_rotation_matrix(self.orientation)
        mat[:3, 3] = self.translation
        return mat


#------------------------------------------------------------------------------
# Packet Unpacker
#------------------------------------------------------------------------------

class _packet:
    def __init__(self, timestamp, payload, pose):
        self.timestamp = timestamp
        self.payload   = payload
        self.pose      = pose


def is_valid_pose(pose):
    if isinstance(pose, Pose):
        return True
    return pose[3, 3] != 0


#------------------------------------------------------------------------------
# Context Manager
#------------------------------------------------------------------------------

class _context_manager:
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()
        pass


#------------------------------------------------------------------------------
# RPC Interface
#------------------------------------------------------------------------------

class rpc_interface(_context_manager):

    def __init__(self, svc_name, session_config, rpc_topic):
        self.svc_name = svc_name
        self.session_config = session_config
        self.rpc_topic = rpc_topic

    @property
    def session(self):
        # lazy property to support multiprocessing - cannot pickle session
        return hl2ss_core.Locator().get_session(self.session_config)

    def make_request_payload(self, **kw):
        return kw, None

    def _call_rpc(self, payload=None, **kw):
        if payload is None:
            kw, payload = self.make_request_payload(**kw)
        elif isinstance(payload, hl2ss_schema.IdlStruct):
            payload = payload.serialize()
        query = "{}?{}".format(self.rpc_topic, "&".join("{0}={1}".format(k,v) for k,v in kw.items()))
        replies = self.session.get(query, zenoh.Queue(), target=QueryTarget.ALL(), value=payload)
        result = []
        for reply in replies.receiver:
            try:
                result.append(reply.ok)
            except:
                log.error("RPC ERROR: '{}'", reply.err.payload.decode("utf-8"))
        if not result:
            log.warning(f"Call to {query} did not get any results.")
        return result


class mgr_rpc_interface(rpc_interface):
    commands = {
        "StartRM": (hl2ss_schema.HL2MGRRequest_StartRM, hl2ss_schema.NullReply),
        "StopRM": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartMC": (hl2ss_schema.HL2MGRRequest_StartMC, hl2ss_schema.NullReply),
        "StopMC": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartPV": (hl2ss_schema.HL2MGRRequest_StartPV, hl2ss_schema.NullReply),
        "StopPV": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartSI": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StopSI": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartRC": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StopRC": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartSM": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StopSM": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartSU": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StopSU": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartVI": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StopVI": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
        "StartEET": (hl2ss_schema.UInt8Request, hl2ss_schema.NullReply),
        "StopEET": (hl2ss_schema.NullRequest, hl2ss_schema.NullReply),
    }

    def __init__(self, session_config, topic_prefix):
        super().__init__("MGR", session_config, topic_prefix + "/rpc/mgr")

    def make_request_payload(self, **kw):
        if "cmd" not in kw:
            log.warning("RPC: called without command")
            return kw, None
        cmd = kw["cmd"]
        payload = None
        if cmd not in self.commands:
            raise ValueError(f"Invalid Command: {cmd}")
        return kw, payload

    def open(self):
        pass

    def close(self):
        pass

    def do_call(self, cmd=None, payload=None, **kw):
        results = self._call_rpc(cmd=cmd, payload=payload, **kw)
        if results:
            if cmd not in self.commands:
                raise ValueError(f"Invalid Command: {cmd}")
            _, handler = self.commands[cmd]
            reply = handler.deserialize(results[0].payload)
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return True
        log.error("RC: rpc call failed")
        return False

    def start_rm(self, request: hl2ss_schema.HL2MGRRequest_StartRM):
        return self.do_call(cmd="StartRM", payload=request)

    def stop_rm(self):
        return self.do_call(cmd="StopRM")

    def start_mc(self, request: hl2ss_schema.HL2MGRRequest_StartMC):
        return self.do_call(cmd="StartMC", payload=request)

    def stop_mc(self):
        return self.do_call(cmd="StopMC")

    def start_pv(self, request: hl2ss_schema.HL2MGRRequest_StartPV):
        return self.do_call(cmd="StartPV", payload=request)

    def stop_pv(self):
        return self.do_call(cmd="StopPV")

    def start_si(self):
        return self.do_call(cmd="StartSI")

    def stop_si(self):
        return self.do_call(cmd="StopSI")

    def start_rc(self):
        return self.do_call(cmd="StartRC")

    def stop_rc(self):
        return self.do_call(cmd="StopRC")

    def start_sm(self):
        return self.do_call(cmd="StartSM")

    def stop_sm(self):
        return self.do_call(cmd="StopSM")

    def start_su(self):
        return self.do_call(cmd="StartSU")

    def stop_su(self):
        return self.do_call(cmd="StopSU")

    def start_vi(self):
        return self.do_call(cmd="StartVI")

    def stop_vi(self):
        return self.do_call(cmd="StopVI")

    def start_eet(self, eye_fps):
        return self.do_call(cmd="StartEET", value=eye_fps)

    def stop_eet(self):
        return self.do_call(cmd="StopEET")




#------------------------------------------------------------------------------
# Receiver Wrappers
#------------------------------------------------------------------------------

class stream_base(_context_manager):

    def __init__(self, svc_name, session_config, config_topic):
        self.svc_name = svc_name
        self.session_config = session_config
        self.config_topic = config_topic
        self.desc = None
        self.sub = None
        self._queue = numpy_ringbuffer.RingBuffer(1024, dtype=np.object)
        self._session = None

    @property
    def session(self):
        # lazy property to support multiprocessing - cannot pickle session
        return hl2ss_core.Locator().get_session(self.session_config)

    def get_desc(self):
        return self.desc

    def configure(self):
        replies = self.session.get(self.config_topic, zenoh.Queue(), target=QueryTarget.BEST_MATCHING())
        have_result = False
        retry_counter = 0
        while True:
            try:
                desc_reply = next(replies).ok
                self.desc = hl2ss_schema.Hololens2StreamDescriptor.deserialize(desc_reply.payload)
                log.info("{0} received stream_config: {1}".format(self.svc_name, self.desc))
                return

            except StopIteration:
                pass
            log.warning("{0} cannot find endpoint at: {1} - waiting ...".format(self.svc_name, self.config_topic))
            time.sleep(2.)
            replies = self.session.get(self.config_topic, zenoh.Queue(), target=QueryTarget.BEST_MATCHING())
            retry_counter += 1
            if retry_counter > 10:
                raise ValueError("No stream endpoint found at: {0}".format(self.config_topic))


    def open(self):
        if self.desc is None:
            self.configure()
        if self.desc is None:
            log.error("{0} error - cannot retrieve stream_topic from configuration.")
            raise ValueError("Cannot Connect")
        log.info("{0} subscribes to stream_topic: {1}".format(self.svc_name, self.desc.stream_topic))
        self.sub = self.session.declare_subscriber(self.desc.stream_topic, zenoh.Queue(bound=1024), reliability=Reliability.RELIABLE())

    def cb_data(self, sample):
        self._queue.append(sample)

    def close(self):
        self.sub.undeclare()

    def get_next_packet(self):
        try:
            sample = next(self.sub.receiver)
            while sample is None:  # do we need a stop criterion?
                sample = next(self.sub.receiver)
            return self.process_sample(sample)
        except StopIteration:
            return None

    def process_sample(self, sample):
        raise NotImplemented



class rx_video_stream(stream_base):
    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2VideoStream.deserialize(sample.payload)
        pose = Pose(value.position, value.orientation).to_matrix()
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, pose)
        d = self.desc
        if d.image_compression == hl2ss_schema.Hololens2ImageCompression.CompressionType_Raw and \
                self.desc.h26x_profile == hl2ss_schema.Hololens2H26xProfile.H26xProfile_None:
            width = self.desc.image_width
            height = self.desc.image_height
            stride = 0
            if d.sensor_type == hl2ss_schema.Hololens2SensorType.PERSONAL_VIDEO and \
                    d.image_format == hl2ss_schema.Hololens2PixelFormat.PixelFormat_NV12:
                stride = get_nv12_stride(width)
            else:
                log.warning("{0} Unhandled uncompressed sensor type: {1}".format(self.svc_name, d.sensor_type))
            if stride > 0:
                p.image = np.asarray(value.image, dtype=np.uint8).reshape((int(height*3/2), stride))[:, :width]
        return p


class rx_pv(rx_video_stream):

    def __init__(self, cfg, topic_prefix):
        super().__init__("PV", cfg, topic_prefix + "/cfg/desc/pv")


class rx_rm_vlc(rx_video_stream):
    
    def __init__(self, cfg, topic_prefix, port: StreamType):
        suffix = None
        if port == StreamType.RM_VLC_LEFTFRONT:
            suffix = "lf"
        elif port == StreamType.RM_VLC_LEFTLEFT:
            suffix = "ll"
        elif port == StreamType.RM_VLC_RIGHTRIGHT:
            suffix = "rr"
        elif port == StreamType.RM_VLC_RIGHTFRONT:
            suffix = "rf"
        super().__init__("VLC_{0}".format(suffix.upper()), cfg, topic_prefix + "/cfg/desc/vlc_{0}".format(suffix))


class rx_rm_depth_ahat(rx_video_stream):
    
    def __init__(self, cfg, topic_prefix):
        super().__init__("ZHT", cfg, topic_prefix + "/cfg/desc/zht")


class rx_rm_depth_longthrow(stream_base):

    def __init__(self, cfg, topic_prefix):
        super().__init__("ZLT-D", cfg, topic_prefix + "/cfg/desc/zlt/depth")

    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2VideoStream.deserialize(sample.payload)
        pose = Pose(value.position, value.orientation).to_matrix()
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, pose)
        d = self.desc
        if d.image_compression == hl2ss_schema.Hololens2ImageCompression.CompressionType_Raw:
            width = self.desc.image_width
            height = self.desc.image_height
            stride = 0
            if d.sensor_type == hl2ss_schema.Hololens2SensorType.RM_DEPTH_LONG_THROW and \
                    d.image_format == hl2ss_schema.Hololens2PixelFormat.PixelFormat_L16:
                stride = width*2
            else:
                log.warning("{0} Unhandled uncompressed sensor type: {1}".format(self.svc_name, d.sensor_type))
            if stride > 0:
                p.image = np.asarray(value.image, dtype=np.uint8).reshape((int(height*3/2), stride))[:, :width]
        return p


class rx_rm_imu_accel(stream_base):

    def __init__(self, cfg, topic_prefix):
        super().__init__("IMU_ACC", cfg, topic_prefix + "/cfg/desc/imu_acc")

    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2ImuAccel.deserialize(sample.payload)
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, np.zeros((4, 4)))
        return p


class rx_rm_imu_gyro(stream_base):
    def __init__(self, cfg, topic_prefix):
        super().__init__("IMU_GYRO", cfg, topic_prefix + "/cfg/desc/imu_gyro")

    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2ImuGyro.deserialize(sample.payload)
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, np.zeros((4, 4)))
        return p


class rx_rm_imu_mag(stream_base):
    def __init__(self, cfg, topic_prefix):
        super().__init__("IMU_MAG", cfg, topic_prefix + "/cfg/desc/imu_mag")

    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2ImuMAg.deserialize(sample.payload)
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, np.zeros((4, 4)))
        return p


class rx_microphone(stream_base):
    def __init__(self, cfg, topic_prefix):
        super().__init__("MIC", cfg, topic_prefix + "/cfg/desc/mic")

    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2AudioStream.deserialize(sample.payload)
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, np.zeros((4, 4)))
        return p


class rx_si(stream_base):

    def __init__(self, cfg, topic_prefix):
        super().__init__("SI", cfg, topic_prefix + "/cfg/desc/si")

    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2HandTracking.deserialize(sample.payload)
        # pose = Pose(value.position, value.orientation).to_matrix()
        pose = None
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, pose)
        return p


class rx_eet(stream_base):
    def __init__(self, cfg, topic_prefix):
        super().__init__("EET", cfg, topic_prefix + "/cfg/desc/eet")

    def process_sample(self, sample):
        if sample is None:
            log.warning("{0} processed empty sample", self.svc_name)
            return

        value = hl2ss_schema.Hololens2EyeTracking.deserialize(sample.payload)
        pose = Pose(value.position, value.orientation).to_matrix()
        ts = hl2ss_core.Time.from_msg(value.header.stamp)
        p = _packet(ts, value, pose)
        return p



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
    return 1 if ((name != 'h264') and (name != 'hevc')) else framerate


def get_video_codec_default_factor(profile):
    name = get_video_codec_name(profile)
    return 4/420 if (name == 'h264') else 1/140 if (name == 'hevc') else 1.0


def get_video_codec_bitrate(width, height, fps, factor):
    return int(width*height*fps*12*factor)


#------------------------------------------------------------------------------
# RM VLC Decoder
#------------------------------------------------------------------------------

class _decode_rm_vlc:
    def __init__(self, profile):
        self.profile = profile

    def create(self):
        self._codec = av.CodecContext.create(get_video_codec_name(self.profile), 'r')

    def decode(self, payload):
        try:
            # this will only decode one packet ...
            for packet in self._codec.parse(payload):
                for frame in self._codec.decode(packet):
                    return frame.to_ndarray()[:Parameters_RM_VLC.HEIGHT, :Parameters_RM_VLC.WIDTH]
        except av.error.InvalidDataError as e:
            log.error(e)
            def print_nalu(bytes):
                print("nalu: " + len(bytes))

            def parse_sps(bytes):
                sps = SPS(bytes)
                sps.print_verbose()

            from h26x_extractor.h26x_parser import H26xParser
            from h26x_extractor.nalutypes import SPS
            p = H26xParser("test.h264", True)
            p.set_callback("nalu", print_nalu)
            p.set_callback("sps", parse_sps)
            p.parse()

        return None


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

    depth = y.astype(np.uint16) * Parameters_RM_DEPTH_AHAT.FACTOR
    ab = np.zeros((Parameters_RM_DEPTH_AHAT.HEIGHT, Parameters_RM_DEPTH_AHAT.WIDTH), dtype=np.uint16)

    ab[:, 0::4] = u * 256
    ab[:, 1::4] = u * 256
    ab[:, 2::4] = v * 256
    ab[:, 3::4] = v * 256

    return _RM_Depth_Frame(depth, ab)


class _decode_rm_depth_ahat:
    def __init__(self, profile):
        self.profile = profile
   
    def create(self):
        self._codec = av.CodecContext.create(get_video_codec_name(self.profile), 'r')

    def decode(self, payload):
        try:
            # this will only decode one packet ...
            for packet in self._codec.parse(bytes(payload.image)):
                for frame in self._codec.decode(packet):
                    return _unpack_rm_depth_ahat_nv12_as_yuv420p(frame.to_ndarray())
        except av.error.InvalidDataError as e:
            log.error(e)
        return None


class _unpack_rm_depth_ahat:
    def create(self):
        pass

    def decode(self, payload):
        depth = np.frombuffer(payload, dtype=np.uint16, offset=0,                                            count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
        ab    = np.frombuffer(payload, dtype=np.uint16, offset=Parameters_RM_DEPTH_AHAT.PIXELS*_SIZEOF.WORD, count=Parameters_RM_DEPTH_AHAT.PIXELS).reshape(Parameters_RM_DEPTH_AHAT.SHAPE)
        depth[depth >= 4090] = 0
        return _RM_Depth_Frame(depth, ab)


def decode_rm_depth_ahat(profile):
    return _unpack_rm_depth_ahat() if (profile == VideoProfile.RAW) else _decode_rm_depth_ahat(profile)


def decode_rm_depth_longthrow(payload):
    composite = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    h, w, _ = composite.shape
    image = composite.view(np.uint16).reshape((2*h, w))
    return _RM_Depth_Frame(image[:h, :], image[h:, :])


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

class _PV_Frame:
    def __init__(self, image, focal_length, principal_point):
        self.image           = image
        self.focal_length    = focal_length
        self.principal_point = principal_point


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
    return _PV_Frame(bytes(payload.image),
                     np.asarray(payload.camera_focal_length, dtype=np.float32),
                     np.asarray(payload.camera_principal_point, dtype=np.float32))


def get_nv12_stride(width):
    return width + ((64 - (width & 63)) & 63)


class _decode_pv:
    def __init__(self, profile):
        self.profile = profile

    def create(self, width, height):
        cn = get_video_codec_name(self.profile)
        self._codec = av.CodecContext.create(cn, 'r')

    def decode(self, payload, format):
        for packet in self._codec.parse(payload):
            try:
                # this will only decode one packet ...
                for frame in self._codec.decode(packet):
                    return frame.to_ndarray(format=format)
            except av.error.InvalidDataError as e:
                log.error(e)
        return None


class _unpack_pv:
    _cv2_nv12_format = {
        'rgb24' : cv2.COLOR_YUV2RGB_NV12,
        'bgr24' : cv2.COLOR_YUV2BGR_NV12,
        'rgba'  : cv2.COLOR_YUV2RGBA_NV12,
        'bgra'  : cv2.COLOR_YUV2BGRA_NV12,
        'gray8' : cv2.COLOR_YUV2GRAY_NV12,
        'nv12'  : None
    }

    def create(self, width, height):
        self.width = width
        self.height = height
        self.stride = get_nv12_stride(width)

    def decode(self, payload, format):
        image = np.frombuffer(payload, dtype=np.uint8).reshape((int(self.height*3/2), self.stride))[:, :self.width]
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
        self._codec = av.CodecContext.create(get_audio_codec_name(self.profile), 'r')

    def decode(self, payload):
        try:
            # this will only decode one packet ...
            for packet in self._codec.parse(payload):
                for frame in self._codec.decode(packet):
                    return frame.to_ndarray()
        except Exception as e:  # too broad
            log.error(e)
        return None


class _unpack_microphone:
    def create(self):
        pass

    def decode(self, payload):
        return np.frombuffer(payload, dtype=np.int16).reshape((1, -1))


def decode_microphone(profile):
    return _unpack_microphone() if (profile == AudioProfile.RAW) else _decode_microphone(profile)


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
    def __init__(self, payload: List[hl2ss_schema.HandJointPose]):
        self._data = payload

    def get_joint_pose(self, joint: int):
        v = self._data[joint]
        return _SI_HandJointPose(to_quat(v.orientation), to_vec3(v.position), v.radius, v.accuracy)


class unpack_si:
    def __init__(self, payload: hl2ss_schema.Hololens2HandTracking):
        self._data = payload
        self._valid = payload.valid

    def is_valid_head_pose(self):
        return (self._valid & _SI_Field.HEAD) != 0

    def is_valid_eye_ray(self):
        return (self._valid & _SI_Field.EYE) != 0

    def is_valid_hand_left(self):
        return (self._valid & _SI_Field.LEFT) != 0

    def is_valid_hand_right(self):
        return (self._valid & _SI_Field.RIGHT) != 0

    def get_head_pose(self):
        return _SI_HeadPose(to_vec3(self._data.head_position),
                            to_vec3(self._data.head_forward),
                            to_vec3(self._data.head_up))

    def get_eye_ray(self):
        return _SI_EyeRay(to_vec3(self._data.gaze_origin), to_vec3(self._data.gaze_direction))

    def get_hand_left(self):
        return _SI_Hand(self._data.left_poses)

    def get_hand_right(self):
        return _SI_Hand(self._data.right_poses)


#------------------------------------------------------------------------------
# EET Unpacker
#------------------------------------------------------------------------------

class unpack_eet:
    def __init__(self, payload):

        self.combined_ray = _SI_EyeRay(to_vec3(payload.c_origin), to_vec3(payload.c_direction))
        self.left_ray = _SI_EyeRay(to_vec3(payload.l_origin), to_vec3(payload.l_direction))
        self.right_ray = _SI_EyeRay(to_vec3(payload.r_origin), to_vec3(payload.r_direction))
        self.left_openness = payload.l_openness
        self.right_openness = payload.r_openness
        self.vergence_distance = payload.vergence_distance

        valid = payload.valid
        self.calibration_valid = valid & 0x01 != 0
        self.combined_ray_valid = valid & 0x02 != 0
        self.left_ray_valid = valid & 0x04 != 0
        self.right_ray_valid = valid & 0x08 != 0
        self.left_openness_valid = valid & 0x10 != 0
        self.right_openness_valid = valid & 0x20 != 0
        self.vergence_distance_valid = valid & 0x40 != 0


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_rm_vlc(rx_rm_vlc):
    def __init__(self, session_config, config_topic, fmt, stream: StreamType):
        super().__init__(session_config, config_topic, stream)
        self.format = fmt
        self._codec = None

    def open(self):
        self.configure()

        profile = map_videoprofile[self.desc.h26x_profile]
        self._codec = decode_rm_vlc(profile)

        self._codec.create()
        super().open()
        self.get_next_packet()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload.raw_image = data.payload.image
        data.payload.image = self._codec.decode(bytes(data.payload.image))
        return data


class rx_decoded_rm_depth_ahat(rx_video_stream):
    def __init__(self, session_config, config_topic, fmt):
        super().__init__("RM_HT", session_config, config_topic)
        self.format = fmt
        self._codec = None

    def open(self):
        self.configure()

        profile = map_videoprofile[self.desc.h26x_profile]
        self._codec = decode_rm_depth_ahat(profile)

        self._codec.create()
        super().open()
        self.get_next_packet()

    def get_next_packet(self):
        data = super().get_next_packet()
        #data.payload = _unpack_rm_depth_ahat(data.payload)
        data.payload.image = self._codec.decode(data.payload)
        return data


class rx_decoded_rm_depth_longthrow(rx_rm_depth_longthrow):
    def __init__(self, host, port, chunk, mode, png_filter):
        super().__init__(host, port, chunk, mode, png_filter)

    def open(self):
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = decode_rm_depth_longthrow(data.payload)
        return data


class rx_decoded_pv(rx_video_stream):
    def __init__(self, session_config, config_topic, fmt):
        super().__init__("PV", session_config, config_topic)
        self.format = fmt
        self._codec = None

    def open(self):
        self.configure()

        profile = map_videoprofile[self.desc.h26x_profile]
        self._codec = decode_pv(profile)

        self._codec.create(self.desc.image_width, self.desc.image_height)
        super().open()
        self.get_next_packet()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = unpack_pv(data.payload)
        data.payload.image = self._codec.decode(data.payload.image, self.format)
        return data


class rx_decoded_microphone(rx_microphone):
    def __init__(self, session_config, config_topic):
        super().__init__("MIC", session_config, config_topic)
        self._codec = None

    def open(self):
        self.configure()
        if self.desc is None:
            raise ValueError("No stream descriptor found.")

        profile = map_audioprofile[self.desc.aac_profile]
        self._codec = decode_microphone(profile)
        self._codec.create()
        super().open()

    def get_next_packet(self):
        data = super().get_next_packet()
        data.payload = self._codec.decode(data.payload)
        return data


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
    ]


def get_port_index(port):
    return port - StreamType.RM_VLC_LEFTFRONT


def get_port_name(port):
    return _PortName.OF[get_port_index(port)]


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


class PV_BacklightCompensationState:
    Disable = 0
    Enable = 1


class ipc_rc(rpc_interface):
    commands = {
        "GetApplicationVersion": (hl2ss_schema.NullRequest, hl2ss_schema.HL2RCResponse_GetApplicationVersion),
        "GetUTCOffset": (hl2ss_schema.UInt32Request, hl2ss_schema.UInt64Reply),
        "SetHSMarkerState": (hl2ss_schema.UInt32Request, hl2ss_schema.NullReply),
        "GetPVSubsystemStatus": (hl2ss_schema.NullRequest, hl2ss_schema.BoolReply),
        "SetPVFocus": (hl2ss_schema.HL2RCRequest_SetPVFocus, hl2ss_schema.NullReply),
        "SetPVVideoTemporalDenoising": (hl2ss_schema.UInt32Request, hl2ss_schema.NullReply),
        "SetPVWhiteBalancePreset": (hl2ss_schema.UInt32Request, hl2ss_schema.NullReply),
        "SetPVWhiteBalanceValue": (hl2ss_schema.UInt32Request, hl2ss_schema.NullReply),
        "SetPVExposure": (hl2ss_schema.HL2RCRequest_SetPVExposure, hl2ss_schema.NullReply),
        "SetPVExposurePriorityVideo": (hl2ss_schema.UInt32Request, hl2ss_schema.NullReply),
        "SetPVIsoSpeed": (hl2ss_schema.HL2RCRequest_SetPVIsoSpeed, hl2ss_schema.NullReply),
        "SetPVBacklightCompensation": (hl2ss_schema.UInt32Request, hl2ss_schema.NullReply),
        "SetPVSceneMode": (hl2ss_schema.UInt32Request, hl2ss_schema.NullReply),
    }

    def __init__(self, session_config, topic_prefix):
        super().__init__("RC", session_config, topic_prefix + "/rpc/ctrl")

    def make_request_payload(self, **kw):
        if "cmd" not in kw:
            log.warning("RC: called without command")
            return kw, None
        cmd = kw["cmd"]
        payload = None

        if cmd not in self.commands:
            raise ValueError(f"Invalid Command: {cmd}")

        return kw, payload

    def open(self):
        pass

    def close(self):
        pass

    def decode_response(self, cmd, payload):
        if cmd not in self.commands:
            raise ValueError(f"Invalid Command: {cmd}")
        _, request = self.commands[cmd]
        return request.deserialize(payload.payload)

    def get_application_version(self):
        cmd = "GetApplicationVersion"
        results = self._call_rpc(cmd=cmd)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return reply.data
        log.error("RC: rpc call failed")

    def get_utc_offset(self, samples):
        cmd = "GetUTCOffset"
        results = self._call_rpc(cmd=cmd, value=samples)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return reply.value
        log.error("RC: rpc call failed")

    def set_hs_marker_state(self, state):
        cmd = "SetHSMarkerState"
        results = self._call_rpc(cmd=cmd, value=state)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def get_pv_subsystem_status(self):
        cmd = "GetPVSubsystemStatus"
        results = self._call_rpc(cmd=cmd)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return reply.value
        log.error("RC: rpc call failed")

    def wait_for_pv_subsystem(self, status):
        while self.get_pv_subsystem_status() != status:
            pass

    def set_pv_focus(self, focusmode, autofocusrange, distance, value, driverfallback):
        cmd = "SetPVFocus"
        results = self._call_rpc(cmd=cmd, focus_mode=focusmode, autofocus_range=autofocusrange,
                                 distance=distance, value=value, disable_driver_fallback=driverfallback)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def set_pv_video_temporal_denoising(self, mode):
        cmd = "SetPVVideoTemporalDenoising"
        results = self._call_rpc(cmd=cmd, value=mode)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def set_pv_white_balance_preset(self, preset):
        cmd = "SetPVWhiteBalancePreset"
        results = self._call_rpc(cmd=cmd, value=preset)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def set_pv_white_balance_value(self, value):
        cmd = "SetPVWhiteBalanceValue"
        results = self._call_rpc(cmd=cmd, value=value)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def set_pv_exposure(self, mode, value):
        cmd = "SetPVExposure"
        results = self._call_rpc(cmd=cmd, mode=mode, value=value)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")
    
    def set_pv_exposure_priority_video(self, enabled):
        cmd = "SetPVExposurePriorityVideo"
        results = self._call_rpc(cmd=cmd, value=enabled)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def set_pv_iso_speed(self, mode, value):
        cmd = "SetPVIsoSpeed"
        results = self._call_rpc(cmd=cmd, mode=mode, value=value)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def set_pv_backlight_compensation(self, state):
        cmd = "SetPVBacklightCompensation"
        results = self._call_rpc(cmd=cmd, value=state)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")

    def set_pv_scene_mode(self, mode):
        cmd = "SetPVSceneMode"
        results = self._call_rpc(cmd=cmd, value=mode)
        if results:
            reply = self.decode_response(cmd, results[0])
            if reply.status == hl2ss_schema.RPCResponseStatus.RPC_STATUS_SUCCESS:
                return
        log.error("RC: rpc call failed")


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

    def add_task(self, id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format, include_vertex_normals, include_bounds):
        self._data.extend(struct.pack('<16sdIIII', id, max_triangles_per_cubic_meter, vertex_position_format, triangle_index_format, vertex_normal_format, (1 if include_vertex_normals else 0) | (2 if include_bounds else 0)))
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
    _CMD_CREATE_OBSERVER       = 0x00
    _CMD_SET_VOLUMES           = 0x01
    _CMD_GET_OBSERVED_SURFACES = 0x02
    _CMD_GET_MESHES            = 0x03

    def __init__(self, host, port):
        self.host = host
        self.port = port
        
    def open(self):
        self._client = _client()
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
        ids = self._client.download(count * 24, ChunkSize.SINGLE_TRANSFER)
        return [_sm_surface_info(ids[(i*24):(i*24+16)], struct.unpack('<Q', ids[(i*24+16):(i*24+24)])[0]) for i in range(0, count)]
    
    def _download_mesh(self):
        header = self._client.download(100, ChunkSize.SINGLE_TRANSFER)

        index, status, vpl, til, vnl = struct.unpack('<IIIII', header[:20])
        scale = header[20:32]
        pose = header[32:96]
        bsz = struct.unpack('<I', header[96:100])[0]

        if (status != 0):
            return index, None
        
        payload = self._client.download(bsz + vpl + til + vnl, ChunkSize.SINGLE_TRANSFER)

        osb_b = 0
        osb_e = osb_b + bsz
        vpd_b = osb_e
        vpd_e = vpd_b + vpl
        tid_b = vpd_e
        tid_e = tid_b + til
        vnd_b = tid_e
        vnd_e = vnd_b + vnl

        bounds           = payload[osb_b:osb_e]
        vertex_positions = payload[vpd_b:vpd_e]
        triangle_indices = payload[tid_b:tid_e]
        vertex_normals   = payload[vnd_b:vnd_e]

        return index, _sm_mesh(scale, pose, bounds, vertex_positions, triangle_indices, vertex_normals)
    
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
    _CMD_CREATE_RECOGNIZER = 0x00
    _CMD_REGISTER_COMMANDS = 0x01
    _CMD_START = 0x02
    _CMD_POP = 0x03
    _CMD_CLEAR = 0x04
    _CMD_STOP = 0x05

    def __init__(self, host, port):
        self.host = host
        self.port = port

    def open(self):
        self._client = _client()
        self._client.open(self.host, self.port)

    def create_recognizer(self):
        command = struct.pack('<B', ipc_vi._CMD_CREATE_RECOGNIZER)
        self._client.sendall(command)

    def register_commands(self, clear, strings):
        command = bytearray()
        command.extend(struct.pack('<BBB', ipc_vi._CMD_REGISTER_COMMANDS, 1 if (clear) else 0, len(strings)))
        for string in strings:
            encoded = string.encode('utf-16')
            command.extend(struct.pack('<H', len(encoded)))
            command.extend(encoded)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.BYTE, ChunkSize.SINGLE_TRANSFER)
        return struct.unpack('<B', data)[0] != 0
    
    def start(self):
        command = struct.pack('<B', ipc_vi._CMD_START)
        self._client.sendall(command)

    def pop(self):
        command = struct.pack('<B', ipc_vi._CMD_POP)
        self._client.sendall(command)
        data = self._client.download(_SIZEOF.DWORD, ChunkSize.SINGLE_TRANSFER)
        count = struct.unpack('<I', data)[0]
        data = self._client.download(32*count, ChunkSize.SINGLE_TRANSFER)
        return [vi_result(data[(i*32):(i*32+4)], data[(i*32+4):(i*32+8)], data[(i*32+8):(i*32+16)], data[(i*32+16):(i*32+24)], data[(i*32+24):(i*32+32)]) for i in range(0, count)]
    
    def clear(self):
        command = struct.pack('<B', ipc_vi._CMD_CLEAR)
        self._client.sendall(command)

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

