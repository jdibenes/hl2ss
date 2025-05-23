
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

public static partial class hl2ss
{
    //------------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------------

    public static class stream_port
    {
        public const ushort RM_VLC_LEFTFRONT = 3800;
        public const ushort RM_VLC_LEFTLEFT = 3801;
        public const ushort RM_VLC_RIGHTFRONT = 3802;
        public const ushort RM_VLC_RIGHTRIGHT = 3803;
        public const ushort RM_DEPTH_AHAT = 3804;
        public const ushort RM_DEPTH_LONGTHROW = 3805;
        public const ushort RM_IMU_ACCELEROMETER = 3806;
        public const ushort RM_IMU_GYROSCOPE = 3807;
        public const ushort RM_IMU_MAGNETOMETER = 3808;
        public const ushort PERSONAL_VIDEO = 3810;
        public const ushort MICROPHONE = 3811;
        public const ushort SPATIAL_INPUT = 3812;
        public const ushort EXTENDED_EYE_TRACKER = 3817;
        public const ushort EXTENDED_AUDIO = 3818;
        public const ushort EXTENDED_VIDEO = 3819;
        public const ushort EXTENDED_DEPTH = 3821;
    }

    public static class ipc_port
    {
        public const ushort REMOTE_CONFIGURATION = 3809;
        public const ushort SPATIAL_MAPPING = 3813;
        public const ushort SCENE_UNDERSTANDING = 3814;
        public const ushort VOICE_INPUT = 3815;
        public const ushort UNITY_MESSAGE_QUEUE = 3816;
        public const ushort GUEST_MESSAGE_QUEUE = 3820;
    }

    public static class chunk_size
    {
        public const ulong RM_VLC = 4096;
        public const ulong RM_DEPTH_AHAT = 4096;
        public const ulong RM_DEPTH_LONGTHROW = 4096;
        public const ulong RM_IMU = 4096;
        public const ulong PERSONAL_VIDEO = 4096;
        public const ulong MICROPHONE = 4096;
        public const ulong SPATIAL_INPUT = 4096;
        public const ulong EXTENDED_EYE_TRACKER = 4096;
        public const ulong EXTENDED_AUDIO = 4096;
        public const ulong EXTENDED_VIDEO = 4096;
        public const ulong EXTENDED_DEPTH = 4096;
        public const ulong SINGLE_TRANSFER = 4096;
    }

    public static class stream_mode
    {
        public const byte MODE_0 = 0;
        public const byte MODE_1 = 1;
        public const byte MODE_2 = 2;
        public const byte MODE_3 = 3;
    }

    public static class video_profile
    {
        public const byte H264_BASE = 0;
        public const byte H264_MAIN = 1;
        public const byte H264_HIGH = 2;
        public const byte H265_MAIN = 3;
        public const byte RAW = 0xFF;
    }

    public static class h26x_level
    {
        public const byte H264_1 = 10;
        public const byte H264_1_b = 11;
        public const byte H264_1_1 = 11;
        public const byte H264_1_2 = 12;
        public const byte H264_1_3 = 13;
        public const byte H264_2 = 20;
        public const byte H264_2_1 = 21;
        public const byte H264_2_2 = 22;
        public const byte H264_3 = 30;
        public const byte H264_3_1 = 31;
        public const byte H264_3_2 = 32;
        public const byte H264_4 = 40;
        public const byte H264_4_1 = 41;
        public const byte H264_4_2 = 42;
        public const byte H264_5 = 50;
        public const byte H264_5_1 = 51;
        public const byte H264_5_2 = 52;
        public const byte H265_1 = 30;
        public const byte H265_2 = 60;
        public const byte H265_2_1 = 63;
        public const byte H265_3 = 90;
        public const byte H265_3_1 = 93;
        public const byte H265_4 = 120;
        public const byte H265_4_1 = 123;
        public const byte H265_5 = 150;
        public const byte H265_5_1 = 153;
        public const byte H265_5_2 = 156;
        public const byte H265_6 = 180;
        public const byte H265_6_1 = 183;
        public const byte H265_6_2 = 186;
        public const byte DEFAULT = 255;
    }

    public static class depth_profile
    {
        public const byte SAME = 0;
        public const byte ZDEPTH = 1;
    }

    public static class audio_profile
    {
        public const byte AAC_12000 = 0;
        public const byte AAC_16000 = 1;
        public const byte AAC_20000 = 2;
        public const byte AAC_24000 = 3;
        public const byte RAW = 0xFF;
    }

    public static class aac_level
    {
        public const byte L2 = 0x29;
        public const byte L4 = 0x2A;
        public const byte L5 = 0x2B;
        public const byte HEV1L2 = 0x2C;
        public const byte HEV1L4 = 0x2E;
        public const byte HEV1L5 = 0x2F;
        public const byte HEV2L2 = 0x30;
        public const byte HEV2L3 = 0x31;
        public const byte HEV2L4 = 0x32;
        public const byte HEV2L5 = 0x33;
    }

    public static class png_filter_mode
    {
        public const byte AUTOMATIC = 0;
        public const byte DISABLE = 1;
        public const byte SUB = 2;
        public const byte UP = 3;
        public const byte AVERAGE = 4;
        public const byte PAETH = 5;
        public const byte ADAPTIVE = 6;
    }

    public static class h26x_encoder_property
    {
        public const ulong CODECAPI_AVEncCommonRateControlMode = 0;
        public const ulong CODECAPI_AVEncCommonQuality = 1;
        public const ulong CODECAPI_AVEncAdaptiveMode = 2;
        public const ulong CODECAPI_AVEncCommonBufferSize = 3;
        public const ulong CODECAPI_AVEncCommonMaxBitRate = 4;
        public const ulong CODECAPI_AVEncCommonMeanBitRate = 5;
        public const ulong CODECAPI_AVEncCommonQualityVsSpeed = 6;
        public const ulong CODECAPI_AVEncH264CABACEnable = 7;
        public const ulong CODECAPI_AVEncH264SPSID = 8;
        public const ulong CODECAPI_AVEncMPVDefaultBPictureCount = 9;
        public const ulong CODECAPI_AVEncMPVGOPSize = 10;
        public const ulong CODECAPI_AVEncNumWorkerThreads = 11;
        public const ulong CODECAPI_AVEncVideoContentType = 12;
        public const ulong CODECAPI_AVEncVideoEncodeQP = 13;
        public const ulong CODECAPI_AVEncVideoForceKeyFrame = 14;
        public const ulong CODECAPI_AVEncVideoMinQP = 15;
        public const ulong CODECAPI_AVLowLatencyMode = 16;
        public const ulong CODECAPI_AVEncVideoMaxQP = 17;
        public const ulong CODECAPI_VideoEncoderDisplayContentType = 18;
        public const ulong HL2SSAPI_VideoMediaIndex = 0xFFFFFFFFFFFFFFFB;
        public const ulong HL2SSAPI_VideoStrideMask = 0xFFFFFFFFFFFFFFFC;
        public const ulong HL2SSAPI_AcquisitionMode = 0xFFFFFFFFFFFFFFFD;
        public const ulong HL2SSAPI_VLCHostTicksOffsetConstant = 0xFFFFFFFFFFFFFFFE;
        public const ulong HL2SSAPI_VLCHostTicksOffsetExposure = 0xFFFFFFFFFFFFFFFF;
    }

    public static class hologram_perspective
    {
        public const uint DISPLAY = 0;
        public const uint PV = 1;
    }

    public static class mixer_mode
    {
        public const uint MICROPHONE = 0;
        public const uint SYSTEM = 1;
        public const uint BOTH = 2;
        public const uint QUERY = 3;
    }

    public static class pv_decoded_format
    {
        public const byte BGR = 0;
        public const byte RGB = 1;
        public const byte BGRA = 2;
        public const byte RGBA = 3;
        public const byte GRAY = 4;
        public const byte ANY = 254;
        public const byte NONE = 255;
    }

    public static class media_category
    {
        public const byte Other = 0;
        public const byte Communications = 1;
        public const byte Media = 2;
        public const byte GameChat = 3;
        public const byte Speech = 4;
    }

    public static class parameters_rm_vlc
    {
        public const ushort WIDTH = 640;
        public const ushort HEIGHT = 480;
        public const byte FPS = 30;
        public const uint PIXELS = WIDTH * HEIGHT;
    }

    public static class parameters_rm_depth_ahat
    {
        public const ushort WIDTH = 512;
        public const ushort HEIGHT = 512;
        public const byte FPS = 45;
        public const uint PIXELS = WIDTH * HEIGHT;
    }

    public static class parameters_rm_depth_longthrow
    {
        public const ushort WIDTH = 320;
        public const ushort HEIGHT = 288;
        public const byte FPS = 5;
        public const uint PIXELS = WIDTH * HEIGHT;
    }

    public static class parameters_rm_imu_accelerometer
    {
        public const ushort BATCH_SIZE = 93;
    }

    public static class parameters_rm_imu_gyroscope
    {
        public const ushort BATCH_SIZE = 315;
    }

    public static class parameters_rm_imu_magnetometer
    {
        public const ushort BATCH_SIZE = 11;
    }

    public static class parameters_microphone
    {
        public const byte CHANNELS = 2;

        public const byte ARRAY_CHANNELS = 5;
        public const byte ARRAY_TOP_LEFT = 0;
        public const byte ARRAY_TOP_CENTER = 1;
        public const byte ARRAY_TOP_RIGHT = 2;
        public const byte ARRAY_BOTTOM_LEFT = 3;
        public const byte ARRAY_BOTTOM_RIGHT = 4;

        public const uint SAMPLE_RATE = 48000;
        public const ushort GROUP_SIZE_RAW = 768;
        public const ushort GROUP_SIZE_AAC = 1024;
    }

    public static class parameters_si
    {
        public const byte SAMPLE_RATE = 30;
    }

    public static class time_base
    {
        public const ulong HUNDREDS_OF_NANOSECONDS = 10000000;
        public const long FILETIME_EPOCH = 0;
        public const long UNIX_EPOCH = 116444736000000000;
    }

    //------------------------------------------------------------------------------
    // Geometry
    //------------------------------------------------------------------------------

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct vector_2
    {
        public float x;
        public float y;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct vector_3
    {
        public float x;
        public float y;
        public float z;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct vector_4
    {
        public float x;
        public float y;
        public float z;
        public float w;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct quaternion
    {
        public float x;
        public float y;
        public float z;
        public float w;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct plane
    {
        public float x;
        public float y;
        public float z;
        public float w;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct matrix_4x4
    {
        public float m_00;
        public float m_01;
        public float m_02;
        public float m_03;
        public float m_10;
        public float m_11;
        public float m_12;
        public float m_13;
        public float m_20;
        public float m_21;
        public float m_22;
        public float m_23;
        public float m_30;
        public float m_31;
        public float m_32;
        public float m_33;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct ray
    {
        public vector_3 origin;
        public vector_3 direction;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct uint64x2
    {
        public ulong val_0;
        public ulong val_1;
    }

    //------------------------------------------------------------------------------
    // Decoder RM VLC
    //------------------------------------------------------------------------------

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct rm_vlc_metadata
    {
        public ulong sensor_ticks;
        public ulong exposure;
        public uint gain;
        public uint _reserved;
    }

    // NO LAYOUT
    public class map_rm_vlc
    {
        public IntPtr image;
        public IntPtr metadata;
    }

    //------------------------------------------------------------------------------
    // Decoder RM Depth
    //------------------------------------------------------------------------------

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct rm_depth_ahat_metadata
    {
        public ulong sensor_ticks;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct rm_depth_longthrow_metadata
    {
        public ulong sensor_ticks;
    }

    // NO LAYOUT
    public class map_rm_depth_ahat
    {
        public IntPtr depth;
        public IntPtr ab;
        public IntPtr metadata;
    }

    // NO LAYOUT
    public class map_rm_depth_longthrow
    {
        public IntPtr depth;
        public IntPtr ab;
        public IntPtr metadata;
    }

    //------------------------------------------------------------------------------
    // Decoder RM IMU
    //------------------------------------------------------------------------------

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct rm_imu_sample
    {
        public ulong sensor_timestamp;
        public ulong timestamp;
        public float x;
        public float y;
        public float z;
        public float temperature;
    }

    // NO LAYOUT
    public class map_rm_imu
    {
        public IntPtr samples;
        public uint count;
    }

    //------------------------------------------------------------------------------
    // Decoder PV
    //------------------------------------------------------------------------------

    public static class pv_focus_state
    {
        public const uint UNINITIALIZED = 0;
        public const uint LOST = 1;
        public const uint SEARCHING = 2;
        public const uint FOCUSED = 3;
        public const uint FAILED = 4;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct pv_metadata
    {
        public vector_2 f;
        public vector_2 c;
        public ulong exposure_time;
        public uint64x2 exposure_compensation;
        public uint lens_position;
        public uint focus_state;
        public uint iso_speed;
        public uint white_balance;
        public vector_2 iso_gains;
        public vector_3 white_balance_gains;
        public ushort width;
        public ushort height;
    }

    // NO LAYOUT
    public class map_pv
    {
        public IntPtr image;
        public IntPtr metadata;
    }

    //------------------------------------------------------------------------------
    // Decoder Microphone
    //------------------------------------------------------------------------------

    // NO LAYOUT
    public class map_microphone
    {
        public IntPtr samples;
        public uint count;
    }

    //------------------------------------------------------------------------------
    // Decoder SI
    //------------------------------------------------------------------------------

    public static class si_valid
    {
        public const uint HEAD = 0x01;
        public const uint EYE = 0x02;
        public const uint LEFT = 0x04;
        public const uint RIGHT = 0x08;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct si_head_pose
    {
        public vector_3 position;
        public vector_3 forward;
        public vector_3 up;
    }

    public static class si_hand_joint_kind
    {
        public const byte Palm = 0;
        public const byte Wrist = 1;
        public const byte ThumbMetacarpal = 2;
        public const byte ThumbProximal = 3;
        public const byte ThumbDistal = 4;
        public const byte ThumbTip = 5;
        public const byte IndexMetacarpal = 6;
        public const byte IndexProximal = 7;
        public const byte IndexIntermediate = 8;
        public const byte IndexDistal = 9;
        public const byte IndexTip = 10;
        public const byte MiddleMetacarpal = 11;
        public const byte MiddleProximal = 12;
        public const byte MiddleIntermediate = 13;
        public const byte MiddleDistal = 14;
        public const byte MiddleTip = 15;
        public const byte RingMetacarpal = 16;
        public const byte RingProximal = 17;
        public const byte RingIntermediate = 18;
        public const byte RingDistal = 19;
        public const byte RingTip = 20;
        public const byte LittleMetacarpal = 21;
        public const byte LittleProximal = 22;
        public const byte LittleIntermediate = 23;
        public const byte LittleDistal = 24;
        public const byte LittleTip = 25;
        public const byte TOTAL = 26;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct si_hand_joint
    {
        public quaternion orientation;
        public vector_3 position;
        public float radius;
        public int accuracy;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct si_frame
    {
        public uint valid;
        public si_head_pose head_pose;
        public ray eye_ray;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 26)]
        public si_hand_joint[] left_hand;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 26)]
        public si_hand_joint[] right_hand;
    }

    // NO LAYOUT
    public class map_si
    {
        public IntPtr tracking;
    }

    //------------------------------------------------------------------------------
    // Decoder EET
    //------------------------------------------------------------------------------

    public static class eet_valid
    {
        public const uint CALIBRATION = 0x01;
        public const uint COMBINED_RAY = 0x02;
        public const uint LEFT_RAY = 0x04;
        public const uint RIGHT_RAY = 0x08;
        public const uint LEFT_OPENNESS = 0x10;
        public const uint RIGHT_OPENNESS = 0x20;
        public const uint VERGENCE_DISTANCE = 0x40;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct eet_frame
    {
        public uint _reserved;
        public ray combined_ray;
        public ray left_ray;
        public ray right_ray;
        public float left_openness;
        public float right_openness;
        public float vergence_distance;
        public uint valid;
    }

    // NO LAYOUT
    public class map_eet
    {
        public IntPtr tracking;
    }

    //------------------------------------------------------------------------------
    // Decoder Extended Depth
    //------------------------------------------------------------------------------

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct extended_depth_metadata
    {
        public ushort width;
        public ushort height;
    }

    // NO LAYOUT
    public class map_extended_depth
    {
        public IntPtr depth;
        public IntPtr metadata;
    }

    //------------------------------------------------------------------------------
    // Mode 2 Data Acquisition
    //------------------------------------------------------------------------------

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct calibration_rm_vlc
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * parameters_rm_vlc.HEIGHT * parameters_rm_vlc.WIDTH)]
        public float[] uv2xy;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * 4)]
        public float[] extrinsics;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * parameters_rm_vlc.HEIGHT * parameters_rm_vlc.WIDTH)]
        public float[] undistort_map;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public float[] intrinsics;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct calibration_rm_depth_ahat
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * parameters_rm_depth_ahat.HEIGHT * parameters_rm_depth_ahat.WIDTH)]
        public float[] uv2xy;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * 4)]
        public float[] extrinsics;
        public float scale;
        public float alias;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * parameters_rm_depth_ahat.HEIGHT * parameters_rm_depth_ahat.WIDTH)]
        public float[] undistort_map;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public float[] intrinsics;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct calibration_rm_depth_longthrow
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * parameters_rm_depth_longthrow.HEIGHT * parameters_rm_depth_longthrow.WIDTH)]
        public float[] uv2xy;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * 4)]
        public float[] extrinsics;
        public float scale;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2 * parameters_rm_depth_longthrow.HEIGHT * parameters_rm_depth_longthrow.WIDTH)]
        public float[] undistort_map;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public float[] intrinsics;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct calibration_rm_imu
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * 4)]
        public float[] extrinsics;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct calibration_pv
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public float[] focal_length;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public float[] principal_point;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] radial_distortion;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public float[] tangential_distortion;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * 4)]
        public float[] projection;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * 4)]
        public float[] extrinsics;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public float[] intrinsics_mf;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public float[] extrinsics_mf;
    }

    //------------------------------------------------------------------------------
    // Remote Configuration
    //------------------------------------------------------------------------------

    public static class hs_marker_state
    {
        public const uint Disable = 0;
        public const uint Enable = 1;
    }

    public static class pv_focus_mode
    {
        public const uint Auto = 0;
        public const uint Single = 1;
        public const uint Continuous = 2;
        public const uint Manual = 3;
    }

    public static class pv_auto_focus_range
    {
        public const uint FullRange = 0;
        public const uint Macro = 1;
        public const uint Normal = 2;
    }

    public static class pv_manual_focus_distance
    {
        public const uint Infinity = 0;
        public const uint Nearest = 2;
    }

    public static class pv_focus_value
    {
        public const uint Min = 170;
        public const uint Max = 10000;
    }

    public static class pv_driver_fallback
    {
        public const uint Enable = 0;
        public const uint Disable = 1;
    }

    public static class pv_video_temporal_denoising_mode
    {
        public const uint Off = 0;
        public const uint On = 1;
    }

    public static class pv_color_temperature_preset
    {
        public const uint Auto = 0;
        public const uint Manual = 1;
        public const uint Cloudy = 2;
        public const uint Daylight = 3;
        public const uint Flash = 4;
        public const uint Fluorescent = 5;
        public const uint Tungsten = 6;
        public const uint Candlelight = 7;
    }

    public static class pv_white_balance_value
    {
        public const uint Min = 2300; // 25
        public const uint Max = 7500; // 25
    }

    public static class pv_exposure_mode
    {
        public const uint Manual = 0;
        public const uint Auto = 1;
    }

    public static class pv_exposure_value
    {
        public const uint Min = 1000; // 10
        public const uint Max = 660000; // 10
    }

    public static class pv_exposure_priority_video
    {
        public const uint Disabled = 0;
        public const uint Enabled = 1;
    }

    public static class pv_iso_speed_mode
    {
        public const uint Manual = 0;
        public const uint Auto = 1;
    }

    public static class pv_iso_speed_value
    {
        public const uint Min = 100;
        public const uint Max = 3200;
    }

    public static class pv_backlight_compensation_state
    {
        public const uint Disable = 0;
        public const uint Enable = 1;
    }

    public static class pv_capture_scene_mode
    {
        public const uint Auto = 0;
        public const uint Macro = 2;
        public const uint Portrait = 3;
        public const uint Sport = 4;
        public const uint Snow = 5;
        public const uint Night = 6;
        public const uint Beach = 7;
        public const uint Sunset = 8;
        public const uint Candlelight = 9;
        public const uint Landscape = 10;
        public const uint NightPortrait = 11;
        public const uint Backlit = 12;
    }

    public static class pv_media_capture_optimization
    {
        public const uint Default = 0;
        public const uint Quality = 1;
        public const uint Latency = 2;
        public const uint Power = 3;
        public const uint LatencyThenQuality = 4;
        public const uint LatencyThenPower = 5;
        public const uint PowerAndQuality = 6;
    }

    public static class pv_capture_use
    {
        public const uint NotSet = 0;
        public const uint Photo = 1;
        public const uint Video = 2;
    }

    public static class pv_optical_image_stabilization_mode
    {
        public const uint Off = 0;
        public const uint On = 1;
    }

    public static class pv_hdr_video_mode
    {
        public const uint Off = 0;
        public const uint On = 1;
        public const uint Auto = 2;
    }

    public static class pv_region_of_interest_weight
    {
        public const uint Min = 0;
        public const uint Max = 100;
    }

    public static class pv_region_of_interest_type
    {
        public const uint Unknown = 0;
        public const uint Face = 1;
    }

    public static class ee_interface_priority
    {
        public const int LOWEST = -2;
        public const int BELOW_NORMAL = -1;
        public const int NORMAL = 0;
        public const int ABOVE_NORMAL = 1;
        public const int HIGHEST = 2;
    }

    public static class rm_map_camera_point_operation
    {
        public const int ImagePointToCameraUnitPlane = 0;
        public const int CameraSpaceToImagePoint = 1;
    }

    public static class ts_source
    {
        public const int QPC = 0;
        public const int UTC = 1;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 2)]
    public class version
    {
        public ushort field_0;
        public ushort field_1;
        public ushort field_2;
        public ushort field_3;
    }

    //------------------------------------------------------------------------------
    // Spatial Mapping
    //------------------------------------------------------------------------------

    public static class sm_vertex_position_format
    {
        public const uint R32G32B32A32Float = 2;
        public const uint R16G16B16A16IntNormalized = 13;
    }

    public static class sm_triangle_index_format
    {
        public const uint R16UInt = 57;
        public const uint R32Uint = 42;
    }

    public static class sm_vertex_normal_format
    {
        public const uint R32G32B32A32Float = 2;
        public const uint R8G8B8A8IntNormalized = 31;
    }

    public static class sm_volume_type
    {
        public const uint Box = 0;
        public const uint Frustum = 1;
        public const uint OrientedBox = 2;
        public const uint Sphere = 3;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct sm_box
    {
        public vector_3 center;
        public vector_3 extents;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct sm_frustum
    {
        public plane p_near;
        public plane p_far;
        public plane p_right;
        public plane p_left;
        public plane p_top;
        public plane p_bottom;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct sm_oriented_box
    {
        public vector_3 center;
        public vector_3 extents;
        public quaternion orientation;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct sm_sphere
    {
        public vector_3 center;
        public float radius;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct guid
    {
        public ulong l;
        public ulong h;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct sm_surface_info
    {
        public guid id;
        public ulong update_time;
    }

    //------------------------------------------------------------------------------
    // Scene Understanding
    //------------------------------------------------------------------------------

    public static class su_mesh_lod
    {
        public const uint Coarse = 0;
        public const uint Medium = 1;
        public const uint Fine = 2;
        public const uint Unlimited = 255;
    }

    public static class su_kind_flag
    {
        public const byte Background = 1;
        public const byte Wall = 2;
        public const byte Floor = 4;
        public const byte Ceiling = 8;
        public const byte Platform = 16;
        public const byte Unknown = 32;
        public const byte World = 64;
        public const byte CompletelyInferred = 128;
    }

    public static class su_create
    {
        public const byte New = 0;
        public const byte NewFromPrevious = 1;
    }

    public static class su_kind
    {
        public const int Background = 0;
        public const int Wall = 1;
        public const int Floor = 2;
        public const int Ceiling = 3;
        public const int Platform = 4;
        public const int Unknown = 247;
        public const int World = 248;
        public const int CompletelyInferred = 249;
    }

    // NO LAYOUT
    public class su_task
    {
        public bool enable_quads;
        public bool enable_meshes;
        public bool enable_only_observed;
        public bool enable_world_mesh;
        public uint mesh_lod;
        public float query_radius;
        public byte create_mode;
        public byte kind_flags;
        public bool get_orientation;
        public bool get_position;
        public bool get_location_matrix;
        public bool get_quad;
        public bool get_meshes;
        public bool get_collider_meshes;
        public guid[] guid_list;
    }

    //------------------------------------------------------------------------------
    // Voice Input
    //------------------------------------------------------------------------------

    public static class vi_speech_recognition_confidence
    {
        public const uint High = 0;
        public const uint Medium = 1;
        public const uint Low = 2;
        public const uint Rejected = 3;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct vi_result
    {
        public uint index;
        public uint confidence;
        public ulong phrase_duration;
        public ulong phrase_start_time;
        public double raw_confidence;
    }

    //------------------------------------------------------------------------------
    // Packer
    //------------------------------------------------------------------------------

    // NO LAYOUT
    public class pointer : IDisposable
    {
        protected GCHandle h;
        protected IntPtr p;

        protected pointer(GCHandle o)
        {
            h = o;
            p = h.AddrOfPinnedObject();
        }

        public static pointer get<T>(T o)
        {
            return new pointer(GCHandle.Alloc(o, GCHandleType.Pinned));
        }

        public IntPtr value { get { return p; } }

        protected virtual void Dispose(bool disposing)
        {
            if (p == IntPtr.Zero) { return; }
            p = IntPtr.Zero;
            h.Free();
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        ~pointer()
        {
            Dispose(false);
        }
    }

    // NO LAYOUT
    public class byte_buffer
    {
        protected List<byte> m_buffer;

        public byte_buffer()
        {
            m_buffer = new List<byte>();
        }

        public void clear()
        {
            m_buffer.Clear();
        }

        public void push_u8(byte v) { m_buffer.Add(v); }
        public void push_s8(char v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_u16(ushort v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_s16(short v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_u32(uint v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_s32(int v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_u64(ulong v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_s64(long v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_float(float v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }
        public void push_double(double v) { m_buffer.AddRange(BitConverter.GetBytes(v)); }

        public void push(IntPtr data, ulong size)
        {
            if (size <= 0) { return; }
            byte[] a = new byte[size];
            Marshal.Copy(data, a, 0, (int)size);
            m_buffer.AddRange(a);
        }

        public void push<T>(T v) where T : struct
        {
            using (pointer p = pointer.get(v)) { push(p.value, (ulong)Marshal.SizeOf<T>()); }
        }

        public void push(byte[] v)
        {
            m_buffer.AddRange(v);
        }

        public byte[] get_data()
        {
            return m_buffer.ToArray();
        }

        public ulong get_size()
        {
            return (ulong)m_buffer.Count;
        }
    }

    //------------------------------------------------------------------------------
    // Geometry
    //------------------------------------------------------------------------------

    public static bool is_valid_pose(matrix_4x4 pose)
    {
        return pose.m_33 != 0.0f;
    }

    //------------------------------------------------------------------------------
    // Stream Configuration
    //------------------------------------------------------------------------------

    public static uint extended_audio_device_mixer_mode(uint mixer_mode, uint device_index, uint source_index, uint format_index)
    {
        return mixer_mode | (((device_index + 1) & 0x3FF) << 2) | ((source_index & 0x3FF) << 12) | ((format_index & 0x3FF) << 22);
    }

    public static byte extended_audio_raw_configuration(byte media_category, bool shared, bool audio_raw, bool disable_effect, bool enable_passthrough)
    {
        byte b0_2 = (byte)(Convert.ToByte(media_category) & 7);
        byte b3_3 = Convert.ToByte(shared);
        byte b4_4 = 0;
        byte b5_5 = Convert.ToByte(audio_raw);
        byte b6_6 = Convert.ToByte(disable_effect);
        byte b7_7 = Convert.ToByte(enable_passthrough);

        return (byte)((b7_7 << 7) | (b6_6 << 6) | (b5_5 << 5) | (b4_4 << 4) | (b3_3 << 3) | b0_2);
    }

    //------------------------------------------------------------------------------
    // RM IMU
    //------------------------------------------------------------------------------

    public static ushort rm_imu_get_batch_size(ushort port)
    {
        switch (port)
        {
        case stream_port.RM_IMU_ACCELEROMETER: return parameters_rm_imu_accelerometer.BATCH_SIZE;
        case stream_port.RM_IMU_GYROSCOPE: return parameters_rm_imu_gyroscope.BATCH_SIZE;
        case stream_port.RM_IMU_MAGNETOMETER: return parameters_rm_imu_magnetometer.BATCH_SIZE;
        default: return 0;
        }
    }

    public static void rm_imu_fix_soc_ticks(rm_imu_sample[] map)
    {
        for (int i = 0; i < map.Length; ++i) { map[i].timestamp += (map[i].sensor_timestamp - map[0].sensor_timestamp) / 100; }
    }

    //------------------------------------------------------------------------------
    // PV
    //------------------------------------------------------------------------------

    public static ushort pv_get_video_stride(ushort width)
    {
        return (ushort)((width + 63) & ~63);
    }

    //------------------------------------------------------------------------------
    // Microphone
    //------------------------------------------------------------------------------

    public static T[] microphone_planar_to_packed<T>(T[] map, uint channels)
    {
        T[] data = new T[map.Length];
        int frames = map.Length / (int)channels;
        for (int i = 0; i < frames; ++i) { for (int j = 0; j < channels; ++j) { data[(i * channels) + j] = map[(j * frames) + i]; } }
        return data;
    }

    public static T[] microphone_packed_to_planar<T>(T[] map, uint channels)
    {
        T[] data = new T[map.Length];
        int frames = map.Length / (int)channels;
        for (int i = 0; i < channels; ++i) { for (int j = 0; j < frames; ++j) { data[(i * frames) + j] = map[(j * channels) + i]; } }
        return data;
    }

    //------------------------------------------------------------------------------
    // SI
    //------------------------------------------------------------------------------

    public static string si_get_joint_name(byte joint_kind)
    {
        switch (joint_kind)
        {
        case si_hand_joint_kind.Palm:               return "Palm";
        case si_hand_joint_kind.Wrist:              return "Wrist";
        case si_hand_joint_kind.ThumbMetacarpal:    return "ThumbMetacarpal";
        case si_hand_joint_kind.ThumbProximal:      return "ThumbProximal";
        case si_hand_joint_kind.ThumbDistal:        return "ThumbDistal";
        case si_hand_joint_kind.ThumbTip:           return "ThumbTip";
        case si_hand_joint_kind.IndexMetacarpal:    return "IndexMetacarpal";
        case si_hand_joint_kind.IndexProximal:      return "IndexProximal";
        case si_hand_joint_kind.IndexIntermediate:  return "IndexIntermediate";
        case si_hand_joint_kind.IndexDistal:        return "IndexDistal";
        case si_hand_joint_kind.IndexTip:           return "IndexTip";
        case si_hand_joint_kind.MiddleMetacarpal:   return "MiddleMetacarpal";
        case si_hand_joint_kind.MiddleProximal:     return "MiddleProximal";
        case si_hand_joint_kind.MiddleIntermediate: return "MiddleIntermediate";
        case si_hand_joint_kind.MiddleDistal:       return "MiddleDistal";
        case si_hand_joint_kind.MiddleTip:          return "MiddleTip";
        case si_hand_joint_kind.RingMetacarpal:     return "RingMetacarpal";
        case si_hand_joint_kind.RingProximal:       return "RingProximal";
        case si_hand_joint_kind.RingIntermediate:   return "RingIntermediate";
        case si_hand_joint_kind.RingDistal:         return "RingDistal";
        case si_hand_joint_kind.RingTip:            return "RingTip";
        case si_hand_joint_kind.LittleMetacarpal:   return "LittleMetacarpal";
        case si_hand_joint_kind.LittleProximal:     return "LittleProximal";
        case si_hand_joint_kind.LittleIntermediate: return "LittleIntermediate";
        case si_hand_joint_kind.LittleDistal:       return "LittleDistal";
        case si_hand_joint_kind.LittleTip:          return "LittleTip";
        default:                                    return null;
        }
    }

    //------------------------------------------------------------------------------
    // Port Information
    //------------------------------------------------------------------------------

    public static string get_port_name(ushort port)
    {
        switch (port)
        {
        case stream_port.RM_VLC_LEFTFRONT:     return "rm_vlc_leftfront";
        case stream_port.RM_VLC_LEFTLEFT:      return "rm_vlc_leftleft";
        case stream_port.RM_VLC_RIGHTFRONT:    return "rm_vlc_rightfront";
        case stream_port.RM_VLC_RIGHTRIGHT:    return "rm_vlc_rightright";
        case stream_port.RM_DEPTH_AHAT:        return "rm_depth_ahat";
        case stream_port.RM_DEPTH_LONGTHROW:   return "rm_depth_longthrow";
        case stream_port.RM_IMU_ACCELEROMETER: return "rm_imu_accelerometer";
        case stream_port.RM_IMU_GYROSCOPE:     return "rm_imu_gyroscope";
        case stream_port.RM_IMU_MAGNETOMETER:  return "rm_imu_magnetometer";
        case ipc_port.REMOTE_CONFIGURATION:    return "remote_configuration";
        case stream_port.PERSONAL_VIDEO:       return "personal_video";
        case stream_port.MICROPHONE:           return "microphone";
        case stream_port.SPATIAL_INPUT:        return "spatial_input";
        case ipc_port.SPATIAL_MAPPING:         return "spatial_mapping";
        case ipc_port.SCENE_UNDERSTANDING:     return "scene_understanding";
        case ipc_port.VOICE_INPUT:             return "voice_input";
        case ipc_port.UNITY_MESSAGE_QUEUE:     return "unity_message_queue";
        case stream_port.EXTENDED_EYE_TRACKER: return "extended_eye_tracker";
        case stream_port.EXTENDED_AUDIO:       return "extended_audio";
        case stream_port.EXTENDED_VIDEO:       return "extended_video";
        case ipc_port.GUEST_MESSAGE_QUEUE:     return "guest_message_queue";
        case stream_port.EXTENDED_DEPTH:       return "extended_depth";
        default:                               return null;
        }
    }

    //------------------------------------------------------------------------------
    // Unpacking
    //------------------------------------------------------------------------------

    public static map_rm_vlc unpack_rm_vlc(IntPtr payload, uint size)
    {
        var r = new map_rm_vlc();
        r.image = payload;
        r.metadata = IntPtr.Add(payload, (int)size - Marshal.SizeOf<rm_vlc_metadata>());
        return r;
    }

    public static map_rm_depth_ahat unpack_rm_depth_ahat(IntPtr payload, uint size)
    {
        var r = new map_rm_depth_ahat();
        r.depth = payload;
        r.ab = IntPtr.Add(payload, (int)parameters_rm_depth_ahat.PIXELS * sizeof(ushort));
        r.metadata = IntPtr.Add(payload, (int)size - Marshal.SizeOf<rm_depth_ahat_metadata>());
        return r;
    }

    public static map_rm_depth_longthrow unpack_rm_depth_longthrow(IntPtr payload, uint size)
    {
        var r = new map_rm_depth_longthrow();
        r.depth = payload;
        r.ab = IntPtr.Add(payload, (int)parameters_rm_depth_longthrow.PIXELS * sizeof(ushort));
        r.metadata = IntPtr.Add(payload, (int)size - Marshal.SizeOf<rm_depth_longthrow_metadata>());
        return r;
    }

    public static map_rm_imu unpack_rm_imu(IntPtr payload, uint size)
    {
        var r = new map_rm_imu();
        r.samples = payload;
        r.count = (uint)((int)size / Marshal.SizeOf<rm_imu_sample>());
        return r;
    }

    public static map_pv unpack_pv(IntPtr payload, uint size)
    {
        var r = new map_pv();
        r.image = payload;
        r.metadata = IntPtr.Add(payload, (int)size - Marshal.SizeOf<pv_metadata>());
        return r;
    }

    public static map_microphone unpack_microphone<T>(IntPtr payload, uint size)
    {
        var r = new map_microphone();
        r.samples = payload;
        r.count = (uint)((int)size / Marshal.SizeOf<T>());
        return r;
    }

    public static map_si unpack_si(IntPtr payload, uint size)
    {
        var r = new map_si();
        r.tracking = payload;
        return r;
    }

    public static map_eet unpack_eet(IntPtr payload, uint size)
    {
        var r = new map_eet();
        r.tracking = payload;
        return r;
    }

    public static map_extended_depth unpack_extended_depth(IntPtr payload, uint size)
    {
        var r = new map_extended_depth();
        r.depth = payload;
        r.metadata = IntPtr.Add(payload, (int)size - Marshal.SizeOf<extended_depth_metadata>());
        return r;
    }

    //------------------------------------------------------------------------------
    // Spatial Mapping
    //------------------------------------------------------------------------------

    // NO LAYOUT
    public class sm_bounding_volume
    {
        private byte_buffer m_b;
        private uint m_count;

        public sm_bounding_volume()
        {
            m_count = 0;
            m_b = new byte_buffer();
        }

        public sm_bounding_volume(uint count, byte[] data)
        {
            m_count = count;
            m_b = new byte_buffer();
            m_b.push(data);
        }

        public void clear()
        {
            m_count = 0;
            m_b.clear();
        }

        public void add_box(sm_box box)
        {
            m_count++;
            m_b.push_u32(sm_volume_type.Box);
            m_b.push(box);
        }

        public void add_frustum(sm_frustum frustum)
        {
            m_count++;
            m_b.push_u32(sm_volume_type.Frustum);
            m_b.push(frustum);
        }

        public void add_oriented_box(sm_oriented_box oriented_box)
        {
            m_count++;
            m_b.push_u32(sm_volume_type.OrientedBox);
            m_b.push(oriented_box);
        }

        public void add_sphere(sm_sphere sphere)
        {
            m_count++;
            m_b.push_u32(sm_volume_type.Sphere);
            m_b.push(sphere);
        }

        public uint get_count()
        {
            return m_count;
        }

        public byte[] get_data()
        {
            return m_b.get_data();
        }

        public ulong get_size()
        {
            return m_b.get_size();
        }
    }

    // NO LAYOUT
    public class sm_mesh_task
    {
        private byte_buffer m_b;
        private uint m_count;

        public sm_mesh_task()
        {
            m_count = 0;
            m_b = new byte_buffer();
        }

        public sm_mesh_task(uint count, byte[] data)
        {
            m_count = count;
            m_b = new byte_buffer();            
            m_b.push(data);
        }

        public void clear()
        {
            m_count = 0;
            m_b.clear();
        }

        public void add_task(guid id, double max_triangles_per_cubic_meter, uint vertex_position_format, uint triangle_index_format, uint vertex_normal_format)
        {
            m_count++;
            m_b.push_u64(id.l);
            m_b.push_u64(id.h);
            m_b.push_double(max_triangles_per_cubic_meter);
            m_b.push_u32(vertex_position_format);
            m_b.push_u32(triangle_index_format);
            m_b.push_u32(vertex_normal_format);
            m_b.push_u32(0);
        }

        public uint get_count()
        {
            return m_count;
        }

        public byte[] get_data()
        {
            return m_b.get_data();
        }

        public ulong get_size()
        {
            return m_b.get_size();
        }
    }

    //------------------------------------------------------------------------------
    // Unity Message Queue
    //------------------------------------------------------------------------------
    
    // NO LAYOUT
    public class umq_command_buffer
    {
        private byte_buffer m_b;
        private uint m_count;

        public umq_command_buffer()
        {
            m_b = new byte_buffer();
            m_count = 0;
        }

        public umq_command_buffer(uint count, byte[] data)
        {
            m_count = count;
            m_b = new byte_buffer();
            m_b.push(data);
        }

        public void clear()
        {
            m_count = 0;
            m_b.clear();            
        }

        public void add(uint id, byte[] data)
        {
            m_b.push_u32(id);
            m_b.push_u32((uint)data.Length);
            m_b.push(data);
            m_count++;
        }

        public uint get_count()
        {
            return m_count;
        }

        public byte[] get_data()
        {
            return m_b.get_data();
        }

        public ulong get_size()
        {
            return m_b.get_size();
        }
    }

    //------------------------------------------------------------------------------
    // Timestamps
    //------------------------------------------------------------------------------

    public static long ts_qpc_to_filetime(long timestamp, long utc_offset)
    {
        return timestamp + utc_offset;
    }

    public static long ts_filetime_to_unix_hns(long timestamp_filetime)
    {
        return timestamp_filetime - time_base.UNIX_EPOCH;
    }

    public static double ts_unix_hns_to_unix(long timestamp_unix_hns)
    {
        return timestamp_unix_hns / (double)time_base.HUNDREDS_OF_NANOSECONDS;
    }

    public static long ts_unix_to_unix_hns(double timestamp_unix)
    {
        return (long)(timestamp_unix * time_base.HUNDREDS_OF_NANOSECONDS);
    }

    public static long ts_unix_hns_to_filetime(long timestamp_unix_hns)
    {
        return timestamp_unix_hns + time_base.UNIX_EPOCH;
    }

    public static long ts_filetime_to_qpc(long timestamp_filetime, long utc_offset)
    {
        return timestamp_filetime - utc_offset;
    }
}
