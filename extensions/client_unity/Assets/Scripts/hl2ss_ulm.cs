
using System;
using System.Runtime.InteropServices;

namespace hl2ss
{
    //------------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------------

    public static class stream_port
    {
        public const ushort RM_VLC_LEFTFRONT     = 3800;
        public const ushort RM_VLC_LEFTLEFT      = 3801;
        public const ushort RM_VLC_RIGHTFRONT    = 3802;
        public const ushort RM_VLC_RIGHTRIGHT    = 3803;
        public const ushort RM_DEPTH_AHAT        = 3804;
        public const ushort RM_DEPTH_LONGTHROW   = 3805;
        public const ushort RM_IMU_ACCELEROMETER = 3806;
        public const ushort RM_IMU_GYROSCOPE     = 3807;
        public const ushort RM_IMU_MAGNETOMETER  = 3808;
        public const ushort PERSONAL_VIDEO       = 3810;
        public const ushort MICROPHONE           = 3811;
        public const ushort SPATIAL_INPUT        = 3812;
        public const ushort EXTENDED_EYE_TRACKER = 3817;
        public const ushort EXTENDED_AUDIO       = 3818;
        public const ushort EXTENDED_VIDEO       = 3819;
    }

    public static class ipc_port
    {
        public const ushort REMOTE_CONFIGURATION = 3809;
        public const ushort SPATIAL_MAPPING      = 3813;
        public const ushort SCENE_UNDERSTANDING  = 3814;
        public const ushort VOICE_INPUT          = 3815;
        public const ushort UNITY_MESSAGE_QUEUE  = 3816;
        public const ushort GUEST_MESSAGE_QUEUE  = 3820;
    }

    public static class chunk_size
    {
        public const ulong RM_VLC               = 4096;
        public const ulong RM_DEPTH_AHAT        = 4096;
        public const ulong RM_DEPTH_LONGTHROW   = 4096;
        public const ulong RM_IMU               = 4096;
        public const ulong PERSONAL_VIDEO       = 4096;
        public const ulong MICROPHONE           = 512;
        public const ulong SPATIAL_INPUT        = 1024;
        public const ulong EXTENDED_EYE_TRACKER = 256;
        public const ulong EXTENDED_AUDIO       = 512;
        public const ulong SINGLE_TRANSFER      = 4096;
    }

    public static class stream_mode
    {
        public const byte MODE_0 = 0;
        public const byte MODE_1 = 1;
        public const byte MODE_2 = 2;
    }

    public static class video_profile
    {
        public const byte H264_BASE = 0;
        public const byte H264_MAIN = 1;
        public const byte H264_HIGH = 2;
        public const byte H265_MAIN = 3;
        public const byte RAW       = 0xFF;
    }

    public static class h26x_level
    {
        public const byte H264_1   = 10;
        public const byte H264_1_b = 11;
        public const byte H264_1_1 = 11;
        public const byte H264_1_2 = 12;
        public const byte H264_1_3 = 13;
        public const byte H264_2   = 20;
        public const byte H264_2_1 = 21;
        public const byte H264_2_2 = 22;
        public const byte H264_3   = 30;
        public const byte H264_3_1 = 31;
        public const byte H264_3_2 = 32;
        public const byte H264_4   = 40;
        public const byte H264_4_1 = 41;
        public const byte H264_4_2 = 42;
        public const byte H264_5   = 50;
        public const byte H264_5_1 = 51;
        public const byte H264_5_2 = 52;
        public const byte H265_1   = 30;
        public const byte H265_2   = 60;
        public const byte H265_2_1 = 63;
        public const byte H265_3   = 90;
        public const byte H265_3_1 = 93;
        public const byte H265_4   = 120;
        public const byte H265_4_1 = 123;
        public const byte H265_5   = 150;
        public const byte H265_5_1 = 153;
        public const byte H265_5_2 = 156;
        public const byte H265_6   = 180;
        public const byte H265_6_1 = 183;
        public const byte H265_6_2 = 186;
        public const byte DEFAULT  = 255;
    }

    public static class depth_profile
    {
        public const byte SAME   = 0;
        public const byte ZDEPTH = 1;
    }

    public static class audio_profile
    {
        public const byte AAC_12000 = 0;
        public const byte AAC_16000 = 1;
        public const byte AAC_20000 = 2;
        public const byte AAC_24000 = 3;
        public const byte RAW       = 0xFF;
    }

    public static class aac_level
    {
        public const byte L2      = 0x29;
        public const byte L4      = 0x2A;
        public const byte L5      = 0x2B;
        public const byte HEV1L2  = 0x2C;
        public const byte HEV1L4  = 0x2E;
        public const byte HEV1L5  = 0x2F;
        public const byte HEV2L2  = 0x30;
        public const byte HEV2L3  = 0x31;
        public const byte HEV2L4  = 0x32;
        public const byte HEV2L5  = 0x33;
    }

    public static class png_filter_mode
    {
        public const byte AUTOMATIC = 0;
        public const byte DISABLE   = 1;
        public const byte SUB       = 2;
        public const byte UP        = 3;
        public const byte AVERAGE   = 4;
        public const byte PAETH     = 5;
        public const byte ADAPTIVE  = 6;
    }

    public static class h26x_encoder_property
    {
        public const ulong CODECAPI_AVEncCommonRateControlMode     = 0;
        public const ulong CODECAPI_AVEncCommonQuality             = 1;
        public const ulong CODECAPI_AVEncAdaptiveMode              = 2;
        public const ulong CODECAPI_AVEncCommonBufferSize          = 3;
        public const ulong CODECAPI_AVEncCommonMaxBitRate          = 4;
        public const ulong CODECAPI_AVEncCommonMeanBitRate         = 5;
        public const ulong CODECAPI_AVEncCommonQualityVsSpeed      = 6;
        public const ulong CODECAPI_AVEncH264CABACEnable           = 7;
        public const ulong CODECAPI_AVEncH264SPSID                 = 8;
        public const ulong CODECAPI_AVEncMPVDefaultBPictureCount   = 9;
        public const ulong CODECAPI_AVEncMPVGOPSize                = 10;
        public const ulong CODECAPI_AVEncNumWorkerThreads          = 11;
        public const ulong CODECAPI_AVEncVideoContentType          = 12;
        public const ulong CODECAPI_AVEncVideoEncodeQP             = 13;
        public const ulong CODECAPI_AVEncVideoForceKeyFrame        = 14;
        public const ulong CODECAPI_AVEncVideoMinQP                = 15;
        public const ulong CODECAPI_AVLowLatencyMode               = 16;
        public const ulong CODECAPI_AVEncVideoMaxQP                = 17;
        public const ulong CODECAPI_VideoEncoderDisplayContentType = 18;
        public const ulong HL2SSAPI_VLCHostTicksOffsetConstant     = 0xFFFFFFFFFFFFFFFE;
        public const ulong HL2SSAPI_VLCHostTicksOffsetExposure     = 0xFFFFFFFFFFFFFFFF;
    }

    public static class hologram_perspective
    {
        public const uint DISPLAY = 0;
        public const uint PV      = 1;
    }

    public static class mixer_mode
    {
        public const uint MICROPHONE = 0;
        public const uint SYSTEM     = 1;
        public const uint BOTH       = 2;
        public const uint QUERY      = 0x80000000;
    }

    public static class pv_decoded_format
    {
        public const byte BGR  = 0;
        public const byte RGB  = 1;
        public const byte BGRA = 2;
        public const byte RGBA = 3;
        public const byte GRAY = 4;
    }

    public static class eet_framerate
    {
        public const byte FPS_30 = 30;
        public const byte FPS_60 = 60;
        public const byte FPS_90 = 90;
    }

    public static class parameters_rm_vlc
    {
        public const ushort WIDTH  = 640;
        public const ushort HEIGHT = 480;
        public const byte   FPS    = 30;
        public const uint   PIXELS = WIDTH * HEIGHT;
    }

    public static class parameters_rm_depth_ahat
    {
        public const ushort WIDTH  = 512;
        public const ushort HEIGHT = 512;
        public const byte   FPS    = 45;
        public const uint   PIXELS = WIDTH * HEIGHT;
    }

    public static class parameters_rm_depth_longthrow
    {
        public const ushort WIDTH  = 320;
        public const ushort HEIGHT = 288;
        public const byte   FPS    = 5;
        public const uint   PIXELS = WIDTH * HEIGHT;
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
        public const byte ARRAY_CHANNELS     = 5;
        public const byte ARRAY_TOP_LEFT     = 0;
        public const byte ARRAY_TOP_CENTER   = 1;
        public const byte ARRAY_TOP_RIGHT    = 2;
        public const byte ARRAY_BOTTOM_LEFT  = 3;
        public const byte ARRAY_BOTTOM_RIGHT = 4;

        public const uint   SAMPLE_RATE    = 48000;
        public const byte   CHANNELS       = 2;
        public const ushort GROUP_SIZE_RAW = 768;
        public const ushort GROUP_SIZE_AAC = 1024;
    }

    public static class parameters_si
    {
        public const byte SAMPLE_RATE = 30;
    }

    public static class parameters_extended_audio
    {
        public const uint   SAMPLE_RATE    = 48000;
        public const byte   CHANNELS       = 2;
        public const ushort GROUP_SIZE_AAC = 1024;
    }

    public class ulm
    {
        //-----------------------------------------------------------------------------
        // Initialize
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern void initialize();

        //-----------------------------------------------------------------------------
        // Open
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern void open_rm_vlc(string host, ushort port, ulong chunk = hl2ss.chunk_size.RM_VLC, byte mode = hl2ss.stream_mode.MODE_1, byte divisor = 1, byte profile = hl2ss.video_profile.H265_MAIN, byte level = hl2ss.h26x_level.DEFAULT, uint bitrate = 0, ulong options_size = 0, ulong[] options_data = null, ulong buffer_size = hl2ss.parameters_rm_vlc.FPS * 10);

        [DllImport("hl2ss_ulm")]
        public static extern void open_rm_depth_ahat(string host, ushort port, ulong chunk = hl2ss.chunk_size.RM_DEPTH_AHAT, byte mode = hl2ss.stream_mode.MODE_1, byte divisor = 1, byte profile_z = hl2ss.depth_profile.SAME, byte profile_ab = hl2ss.video_profile.H265_MAIN, byte level = hl2ss.h26x_level.DEFAULT, uint bitrate = 0, ulong options_size = 0, ulong[] options_data = null, ulong buffer_size = hl2ss.parameters_rm_depth_ahat.FPS * 10);

        [DllImport("hl2ss_ulm")]
        public static extern void open_rm_depth_longthrow(string host, ushort port, ulong chunk = hl2ss.chunk_size.RM_DEPTH_LONGTHROW, byte mode = hl2ss.stream_mode.MODE_1, byte divisor = 1, byte png_filter = hl2ss.png_filter_mode.PAETH, ulong buffer_size = hl2ss.parameters_rm_depth_longthrow.FPS * 10);

        [DllImport("hl2ss_ulm")]
        public static extern void open_rm_imu(string host, ushort port, ulong chunk = hl2ss.chunk_size.RM_IMU, byte mode = hl2ss.stream_mode.MODE_1, ulong buffer_size = 300);

        [DllImport("hl2ss_ulm")]
        public static extern void open_pv(string host, ushort port, ushort width, ushort height, byte framerate, ulong chunk = hl2ss.chunk_size.PERSONAL_VIDEO, byte mode = hl2ss.stream_mode.MODE_1, byte divisor = 1, byte profile = hl2ss.video_profile.H265_MAIN, byte level = hl2ss.h26x_level.DEFAULT, uint bitrate = 0, ulong options_size = 0, ulong[] options_data = null, byte decoded_format = hl2ss.pv_decoded_format.BGR, ulong buffer_size = 300);

        [DllImport("hl2ss_ulm")]
        public static extern void open_microphone(string host, ushort port, ulong chunk = hl2ss.chunk_size.MICROPHONE, byte profile = hl2ss.audio_profile.AAC_24000, byte level = hl2ss.aac_level.L2, ulong buffer_size = 500);

        [DllImport("hl2ss_ulm")]
        public static extern void open_si(string host, ushort port, ulong chunk = hl2ss.chunk_size.SPATIAL_INPUT, ulong buffer_size = 300);

        [DllImport("hl2ss_ulm")]
        public static extern void open_eet(string host, ushort port, ulong chunk = hl2ss.chunk_size.EXTENDED_EYE_TRACKER, byte framerate = hl2ss.eet_framerate.FPS_30, ulong buffer_size = 900);

        [DllImport("hl2ss_ulm")]
        public static extern void open_extended_audio(string host, ushort port, ulong chunk = hl2ss.chunk_size.EXTENDED_AUDIO, uint mixer_mode = hl2ss.mixer_mode.BOTH, float loopback_gain = 1.0f, float microphone_gain = 1.0f, byte profile = hl2ss.audio_profile.AAC_24000, byte level = hl2ss.aac_level.L2, ulong buffer_size = 500);

        [DllImport("hl2ss_ulm")]
        public static extern void open_rc(string host, ushort port);

        [DllImport("hl2ss_ulm")]
        public static extern void open_sm(string host, ushort port);

        [DllImport("hl2ss_ulm")]
        public static extern void open_su(string host, ushort port);

        [DllImport("hl2ss_ulm")]
        public static extern void open_vi(string host, ushort port);

        [DllImport("hl2ss_ulm")]
        public static extern void open_umq(string host, ushort port);

        [DllImport("hl2ss_ulm")]
        public static extern void open_gmq(string host, ushort port);

        //-----------------------------------------------------------------------------
        // Close
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern void close(ushort port);

        //-----------------------------------------------------------------------------
        // Grab
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int get_by_index(ushort port, ref long frame_stamp, ref int status, ref IntPtr frame, uint error_size = 0, byte[] error_data = null);

        [DllImport("hl2ss_ulm")]
        public static extern int get_by_timestamp(ushort port, ulong timestamp, int time_preference, int tiebreak_right, ref long frame_stamp, ref int status, ref IntPtr frame, uint error_size = 0, byte[] error_data = null);

        [DllImport("hl2ss_ulm")]
        public static extern void release_frame(IntPtr frame);

        [DllImport("hl2ss_ulm")]
        public static extern int unpack_frame(IntPtr frame, ref ulong timestamp, ref uint payload_size, ref IntPtr payload_data, ref IntPtr pose_data);

        //------------------------------------------------------------------------------
        // Unpacking
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_rm_vlc(IntPtr payload, ref IntPtr image);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_rm_depth_ahat(IntPtr payload, ref IntPtr depth, ref IntPtr ab);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_rm_depth_longthrow(IntPtr payload, ref IntPtr depth, ref IntPtr ab);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_rm_imu(IntPtr payload, ref IntPtr samples);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_pv(IntPtr payload, ulong size, ref IntPtr image, ref IntPtr intrinsics);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_microphone_raw(IntPtr payload, ref IntPtr samples);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_microphone_aac(IntPtr payload, ref IntPtr samples);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_si(IntPtr payload, ref IntPtr si);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_eet(IntPtr payload, ref IntPtr eet);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_extended_audio_raw(IntPtr payload, ref IntPtr samples);

        [DllImport("hl2ss_ulm")]
        public static extern void unpack_extended_audio_aac(IntPtr payload, ref IntPtr samples);

        //------------------------------------------------------------------------------
        // Stream Configuration
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern uint extended_audio_device_mixer_mode(uint mixer_mode, uint device);

        //-----------------------------------------------------------------------------
        // Control
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int start_subsystem_pv(string host, ushort port, byte enable_mrc = 0, byte hologram_composition = 1, byte recording_indicator = 0, byte video_stabilization = 0, byte blank_protected = 0, byte show_mesh = 0, byte shared = 0, float global_opacity = 0.9f, float output_width = 0.0f, float output_height = 0.0f, uint video_stabilization_length = 0, uint hologram_perspective = hl2ss.hologram_perspective.PV, uint error_size = 0, byte[] error_data = null);

        [DllImport("hl2ss_ulm")]
        public static extern int stop_subsystem_pv(string host, ushort port, uint error_size = 0, byte[] error_data = null);

        //-----------------------------------------------------------------------------
        // Calibration
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int download_calibration_rm_vlc(string host, ushort port, IntPtr uv2xy, IntPtr extrinsics, IntPtr undistort_map, IntPtr intrinsics, uint error_size = 0, byte[] error_data = null);

        [DllImport("hl2ss_ulm")]
        public static extern int download_calibration_rm_depth_ahat(string host, ushort port, IntPtr uv2xy, IntPtr extrinsics, IntPtr scale, IntPtr alias, IntPtr undistort_map, IntPtr intrinsics, uint error_size = 0, byte[] error_data = null);

        [DllImport("hl2ss_ulm")]
        public static extern int download_calibration_rm_depth_longthrow(string host, ushort port, IntPtr uv2xy, IntPtr extrinsics, IntPtr scale, IntPtr undistort_map, IntPtr intrinsics, uint error_size = 0, byte[] error_data = null);

        [DllImport("hl2ss_ulm")]
        public static extern int download_calibration_rm_imu(string host, ushort port, IntPtr extrinsics, uint error_size = 0, byte[] error_data = null);

        [DllImport("hl2ss_ulm")]
        public static extern int download_calibration_pv(string host, ushort port, ushort width, ushort height, byte framerate, IntPtr focal_length, IntPtr principal_point, IntPtr radial_distortion, IntPtr tangential_distortion, IntPtr projection, IntPtr extrinsics, uint error_size = 0, byte[] error_data = null);

        //-----------------------------------------------------------------------------
        // IPC
        //-----------------------------------------------------------------------------
    }
}
