from enum import Enum, auto
from dataclasses import dataclass, field
from pycdr2 import IdlStruct, IdlEnum
from pycdr2.types import int32, uint8, uint16, uint32, uint64, float32, float64, sequence, array


class RPCResponseStatus(IdlEnum, typename="RPCResponseStatus"):
    RPC_STATUS_SUCCESS = auto()
    RPC_STATUS_ERROR = auto()


class Hololens2PixelFormat(IdlEnum, typename="Hololens2PixelFormat"):
    PixelFormat_NV12 = auto()
    PixelFormat_ARGB = auto()
    PixelFormat_L8 = auto()
    PixelFormat_L16 = auto()


class Hololens2ImageCompression(IdlEnum, typename="Hololens2ImageCompression"):
    CompressionType_Raw = auto()
    CompressionType_Png = auto()
    CompressionType_H26x = auto()
    CompressionType_Zdepth = auto()


class Hololens2H26xProfile(IdlEnum, typename="Hololens2H26xProfile"):
    H264Profile_Base = auto()
    H264Profile_Main = auto()
    H264Profile_High = auto()
    H265Profile_Main = auto()
    H26xProfile_None = auto()


class Hololens2AACProfile(IdlEnum, typename="Hololens2AACProfile"):
    AACProfile_12000 = auto()
    AACProfile_16000 = auto()
    AACProfile_20000 = auto()
    AACProfile_24000 = auto()
    AACProfile_None = auto()


class Hololens2SensorType(IdlEnum, typename="Hololens2SensorType"):
    PERSONAL_VIDEO = auto()
    EYE_TRACKING = auto()
    MICROPHONE = auto()
    RM_LEFT_FRONT = auto()
    RM_LEFT_LEFT = auto()
    RM_RIGHT_FRONT = auto()
    RM_RIGHT_RIGHT = auto()
    RM_DEPTH_AHAT = auto()
    RM_DEPTH_LONG_THROW = auto()
    RM_IMU_ACCEL = auto()
    RM_IMU_GYRO = auto()
    RM_IMU_MAG = auto()


class Hololens2LogLevel(IdlEnum, typename="Hololens2LogLevel"):
    HL2_LOG_ERROR = auto()
    HL2_LOG_WARNING = auto()
    HL2_LOG_INFO = auto()
    HL2_LOG_DEBUG = auto()
    HL2_LOG_TRACE = auto()


@dataclass
class Vector3(IdlStruct, typename="Vector3"):
    x: float64
    y: float64
    z: float64


@dataclass
class Quaternion(IdlStruct, typename="Quaternion"):
    x: float64
    y: float64
    z: float64
    w: float64


@dataclass
class Time(IdlStruct, typename="Time"):
    sec: int32
    nanosec: uint32


@dataclass
class Duration(IdlStruct, typename="Duration"):
    sec: int32
    nanosec: uint32


@dataclass
class Header(IdlStruct, typename="Header"):
    stamp: Time
    frame_id: str


@dataclass
class Hololens2LogItem(IdlStruct, typename="Hololens2LogItem"):
    timestamp: Time
    severity: Hololens2LogLevel
    message: str


@dataclass
class Hololens2LogMessage(IdlStruct, typename="Hololens2LogMessage"):
    header: Header
    items: sequence[Hololens2LogItem]


# RPC Messages

@dataclass
class NullRequest(IdlStruct, typename="NullRequest"):
    dummy: uint8


@dataclass
class NullReply(IdlStruct, typename="NullReply"):
    status: RPCResponseStatus


@dataclass
class StringRequest(IdlStruct, typename="StringRequest"):
    value: str


@dataclass
class StringReply(IdlStruct, typename="StringReply"):
    value: str
    status: RPCResponseStatus


@dataclass
class BoolRequest(IdlStruct, typename="BoolRequest"):
    value: bool


@dataclass
class BoolReply(IdlStruct, typename="BoolReply"):
    value: bool
    status: RPCResponseStatus


@dataclass
class UInt8Request(IdlStruct, typename="UInt8Request"):
    value: uint8


@dataclass
class UInt8Reply(IdlStruct, typename="UInt8Reply"):
    value: uint8
    status: RPCResponseStatus


@dataclass
class UInt32Request(IdlStruct, typename="UInt32Request"):
    value: uint32


@dataclass
class UInt32Reply(IdlStruct, typename="UInt32Reply"):
    value: uint32
    status: RPCResponseStatus


@dataclass
class UInt64Request(IdlStruct, typename="UInt64Request"):
    value: uint64


@dataclass
class UInt64Reply(IdlStruct, typename="UInt64Reply"):
    value: uint64
    status: RPCResponseStatus


@dataclass
class HL2RCResponse_GetApplicationVersion(IdlStruct, typename="HL2RCResponse_GetApplicationVersion"):
    data: array(uint16, 4)
    status: RPCResponseStatus


@dataclass
class HL2RCRequest_SetPVFocus(IdlStruct, typename="HL2RCRequest_SetPVFocus"):
    focus_mode: uint32
    autofocus_range: uint32
    distance: uint32
    value: uint32
    disable_driver_fallback: uint32


@dataclass
class HL2RCRequest_SetPVExposure(IdlStruct, typename="HL2RCRequest_SetPVExposure"):
    mode: uint32
    value: uint32


@dataclass
class HL2RCRequest_SetPVIsoSpeed(IdlStruct, typename="HL2RCRequest_SetPVIsoSpeed"):
    setauto: uint32
    value: uint32


# Stream and Config Messages

@dataclass
class Hololens2Presence(IdlStruct, typename="Hololens2Presence"):
    header: Header
    heart_beat_counter: uint64


@dataclass
class Hololens2AudioStream(IdlStruct, typename="Hololens2AudioStream"):
    header: Header

    profile: Hololens2AACProfile

    data_size: uint64
    data: sequence[uint8]


@dataclass
class Hololens2AudioStream(IdlStruct, typename="Hololens2AudioStream"):
    header: Header

    profile: Hololens2AACProfile

    data_size: uint64
    data: sequence[uint8]


@dataclass
class Hololens2EyeTracking(IdlStruct, typename="Hololens2EyeTracking"):
    header: Header

    position: Vector3
    orientation: Quaternion

    c_origin: Vector3
    c_direction: Vector3
    l_origin: Vector3
    l_direction: Vector3
    r_origin: Vector3
    r_direction: Vector3

    l_openness: float32
    r_openness: float32

    vergence_distance: float32
    valid: uint32


@dataclass
class HandJointPose(IdlStruct, typename="HandJointPose"):
    orientation: Quaternion
    position: Vector3
    radius: float32
    accuracy: int32


@dataclass
class Hololens2HandTracking(IdlStruct, typename="Hololens2HandTracking"):
    header: Header

    head_position: Vector3
    head_forward: Vector3
    head_up: Vector3

    gaze_origin: Vector3
    gaze_direction: Vector3

    left_poses: sequence[HandJointPose]
    right_poses: sequence[HandJointPose]

    valid: uint8


@dataclass
class Hololens2StreamDescriptor(IdlStruct, typename="Hololens2StreamDescriptor"):
    stream_topic: str
    calib_topic: str
    sensor_type: Hololens2SensorType
    frame_rate: uint32

    position: Vector3
    orientation: Quaternion

    image_height: uint32
    image_width: uint32
    image_step: uint32
    image_format: Hololens2PixelFormat
    image_compression: Hololens2ImageCompression

    h26x_profile: Hololens2H26xProfile
    h26x_bitrate: uint32

    audio_channels: uint8
    aac_profile: Hololens2AACProfile


@dataclass
class Hololens2SensorInfoVLC(IdlStruct, typename="Hololens2SensorInfoVLC"):
    header: Header

    uv2x: sequence[float32]
    uv2y: sequence[float32]

    mapx: sequence[float32]
    mapy: sequence[float32]

    K: array(float32, 4)

    position: Vector3
    orientation: Quaternion


@dataclass
class Hololens2SensorIntrinsicsPV(IdlStruct, typename="Hololens2SensorIntrinsicsPV"):
    header: Header

    width: uint16
    height: uint16

    focal_length: array(float32, 2)
    principal_point: array(float32, 2)

    radial_distortion: array(float32, 3)
    tangential_distortion: array(float32, 2)

    undistorted_projection_transform: array(float32, 16)


@dataclass
class Hololens2ImuAccel(IdlStruct, typename="Hololens2ImuAccel"):
    header: Header

    vinyl_hup_ticks: uint64
    soc_ticks: uint64
    values: array(float32, 3)
    temperature: float32

    position: Vector3
    orientation: Quaternion


@dataclass
class Hololens2ImuGyro(IdlStruct, typename="Hololens2ImuGyro"):
    header: Header

    vinyl_hup_ticks: uint64
    soc_ticks: uint64
    values: array(float32, 3)
    temperature: float32

    position: Vector3
    orientation: Quaternion


@dataclass
class Hololens2ImuMag(IdlStruct, typename="Hololens2ImuMag"):
    header: Header

    vinyl_hup_ticks: uint64
    soc_ticks: uint64
    values: array(float32, 3)

    position: Vector3
    orientation: Quaternion


@dataclass
class Hololens2VideoStream(IdlStruct, typename="Hololens2VideoStream"):
    header: Header

    position: Vector3
    orientation: Quaternion

    camera_focal_length: array(float32, 2)
    camera_principal_point: array(float32, 2)

    camera_radial_distortion: array(float32, 3)
    camera_tangential_distortion: array(float32, 2)

    image_bytes: uint64
    image: sequence[uint8]


# Specific RPC Interfaces

@dataclass
class Hololens2Presence(IdlStruct, typename="Hololens2Presence"):
    header: Header
    heart_beat_counter: uint64


@dataclass
class HL2MGR_H26xFormat(IdlStruct, typename="HL2MGR_H26xFormat"):
    width: uint16
    height: uint16
    frame_rate: uint8
    profile: Hololens2H26xProfile
    bitrate: uint32


@dataclass
class HL2MGR_AACFormat(IdlStruct, typename="HL2MGR_AACFormat"):
    channels: uint32
    sample_rate: uint32
    profile: Hololens2AACProfile


@dataclass
class HL2MGRRequest_StartRM(IdlStruct, typename="HL2MGRRequest_StartRM"):
    enable_location: bool = True
    enable_left_front: bool = False
    enable_left_left: bool = False
    enable_right_front: bool = False
    enable_right_right: bool = False
    vlc_format: HL2MGR_H26xFormat = field(
        default_factory=lambda: HL2MGR_H26xFormat(width=1280, height=720, frame_rate=15,
                                                  profile=Hololens2H26xProfile.H264Profile_High,
                                                  bitrate=20 * 1024 * 1024))
    enable_depth_ahat: bool = False
    enable_depth_long_throw: bool = False
    depth_format: HL2MGR_H26xFormat = field(
        default_factory=lambda: HL2MGR_H26xFormat(width=512, height=512, frame_rate=45,
                                                  profile=Hololens2H26xProfile.H265Profile_Main,
                                                  bitrate=8 * 1024 * 1024))
    enable_imu_accel: bool = False
    enable_imu_gyro: bool = False
    enable_imu_mag: bool = False


@dataclass
class HL2MGRRequest_StartPV(IdlStruct, typename="HL2MGRRequest_StartPV"):
    enable_location: bool
    pv_format: HL2MGR_H26xFormat


@dataclass
class HL2MGRRequest_StartMC(IdlStruct, typename="HL2MGRRequest_StartMC"):
    aac_format: HL2MGR_AACFormat
