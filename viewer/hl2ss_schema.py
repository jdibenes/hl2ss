from enum import Enum, auto
from dataclasses import dataclass
from pycdr2 import IdlStruct, IdlEnum
from pycdr2.types import int8, int32, uint8, uint16, uint32, uint64, float32, float64, sequence, array, case, default


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

    left_poses: sequence[HandJointPose]
    right_poses: sequence[HandJointPose]

    valid: uint8


@dataclass
class Hololens2StreamDescriptor(IdlStruct, typename="Hololens2StreamDescriptor"):

    stream_topic: str
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
