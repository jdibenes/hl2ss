
using System;
using System.Runtime.InteropServices;

public static partial class hl2da
{
    public enum SENSOR_ID
    {
        RM_VLC_LEFTFRONT,
        RM_VLC_LEFTLEFT,
        RM_VLC_RIGHTFRONT,
        RM_VLC_RIGHTRIGHT,
        RM_DEPTH_AHAT,
        RM_DEPTH_LONGTHROW,
        RM_IMU_ACCELEROMETER,
        RM_IMU_GYROSCOPE,
        RM_IMU_MAGNETOMETER,
        PV,
        MICROPHONE,
        SPATIAL_INPUT,
        EXTENDED_EYE_TRACKING,
        EXTENDED_AUDIO,
        EXTENDED_VIDEO,
    };

    public enum MICROPHONE_ARRAY_CHANNEL
    {
        TOP_LEFT = 0,
        TOP_CENTER = 1,
        TOP_RIGHT = 2,
        BOTTOM_LEFT = 3,
        BOTTOM_RIGHT = 4,
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct AccelDataStruct
    {
        [FieldOffset( 0)] public ulong VinylHupTicks;
        [FieldOffset( 8)] public ulong SocTicks;
        [FieldOffset(16)] public float x;
        [FieldOffset(20)] public float y;
        [FieldOffset(24)] public float z;
        [FieldOffset(28)] public float temperature;
        // 32
    };

    [StructLayout(LayoutKind.Explicit)]
    public struct GyroDataStruct
    {
        [FieldOffset( 0)] public ulong VinylHupTicks;
        [FieldOffset( 8)] public ulong SocTicks;
        [FieldOffset(16)] public float x;
        [FieldOffset(20)] public float y;
        [FieldOffset(24)] public float z;
        [FieldOffset(28)] public float temperature;
        // 32
    };

    [StructLayout(LayoutKind.Explicit)]
    public struct MagDataStruct
    {
        [FieldOffset( 0)] public ulong VinylHupTicks;
        [FieldOffset( 8)] public ulong SocTicks;
        [FieldOffset(16)] public float x;
        [FieldOffset(20)] public float y;
        [FieldOffset(24)] public float z;
        [FieldOffset(28)] private float _pad;
        // 32
    };

    public enum SI_HandJointKind
    {
        Palm = 0,
        Wrist = 1,
        ThumbMetacarpal = 2,
        ThumbProximal = 3,
        ThumbDistal = 4,
        ThumbTip = 5,
        IndexMetacarpal = 6,
        IndexProximal = 7,
        IndexIntermediate = 8,
        IndexDistal = 9,
        IndexTip = 10,
        MiddleMetacarpal = 11,
        MiddleProximal = 12,
        MiddleIntermediate = 13,
        MiddleDistal = 14,
        MiddleTip = 15,
        RingMetacarpal = 16,
        RingProximal = 17,
        RingIntermediate = 18,
        RingDistal = 19,
        RingTip = 20,
        LittleMetacarpal = 21,
        LittleProximal = 22,
        LittleIntermediate = 23,
        LittleDistal = 24,
        LittleTip = 25,
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct JointPose
    {
        [FieldOffset( 0)] public float rx;
        [FieldOffset( 4)] public float ry;
        [FieldOffset( 8)] public float rz;
        [FieldOffset(12)] public float rw;
        [FieldOffset(16)] public float tx;
        [FieldOffset(20)] public float ty;
        [FieldOffset(24)] public float tz;
        [FieldOffset(28)] public float radius;
        [FieldOffset(32)] public int accuracy;
        // 36
    }

    public enum STATUS
    {
        DISCARDED = -1,
        OK = 0,
        WAIT = 1,
    }

    public enum TIME_PREFERENCE
    {
        PAST = -1,
        NEAREST = 0,
        FUTURE = 1,
    }

    public enum MC_CHANNELS
    {
        USE_2 = 0,
        USE_5 = 1,
    }

    public enum EE_FPS_INDEX
    {
        FPS_30 = 0,
        FPS_60 = 1,
        FPS_90 = 2,
    }

    [Flags]
    public enum SI_VALID
    {
        HEAD = 1,
        EYE = 2,
        LEFT = 4,
        RIGHT = 8,
    }

    [Flags]
    public enum EE_VALID
    {
        CALIBRATION = 1,
        COMBINED_GAZE = 2,
        LEFT_GAZE = 4,
        RIGHT_GAZE = 8,
        LEFT_OPENNESS = 16,
        RIGHT_OPENNESS = 32,
        VERGENCE_DISTANCE = 64,
    }

    public enum HOLOGRAM_PERSPECTIVE
    {
        DISPLAY = 0,
        PV = 1,
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct pv_captureformat
    {
        [FieldOffset( 0)] public byte enable_mrc;
        [FieldOffset( 1)] public byte hologram_composition;
        [FieldOffset( 2)] public byte recording_indicator;
        [FieldOffset( 3)] public byte video_stabilization;
        [FieldOffset( 4)] public byte blank_protected;
        [FieldOffset( 5)] public byte show_mesh;
        [FieldOffset( 6)] public byte shared;
        [FieldOffset( 7)] private byte _reserved_0;
        [FieldOffset( 8)] public float global_opacity;
        [FieldOffset(12)] public float output_width;
        [FieldOffset(16)] public float output_height;
        [FieldOffset(20)] public uint video_stabilization_length;
        [FieldOffset(24)] public uint hologram_perspective;
        [FieldOffset(28)] public ushort width;
        [FieldOffset(30)] public ushort height;
        [FieldOffset(32)] public byte framerate;
        [FieldOffset(33)] private byte _pad_0;
        [FieldOffset(34)] private ushort _reserved_1;
        // 36
    }

    public enum MIXER_MODE
    {
        MICROPHONE = 0,
        SYSTEM = 1,
        BOTH = 2,
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct ea_captureformat
    {
        [FieldOffset(0)] public uint mixer_mode;
        [FieldOffset(4)] public float loopback_gain;
        [FieldOffset(8)] public float microphone_gain;
        // 12
    };

    [StructLayout(LayoutKind.Explicit, CharSet = CharSet.Unicode)]
    public struct ea_audioformat
    {
        [FieldOffset( 0)] public uint bitrate;
        [FieldOffset( 4)] public uint bits_per_sample;
        [FieldOffset( 8)] public uint channel_count;
        [FieldOffset(12)] public uint sample_rate;
        [FieldOffset(16), MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)] public string subtype;
        // 144
    }
    
    [StructLayout(LayoutKind.Explicit, CharSet = CharSet.Unicode)]
    public struct ev_captureformat
    {
        [FieldOffset(  0)] public byte enable_mrc;
        [FieldOffset(  1)] public byte hologram_composition;
        [FieldOffset(  2)] public byte recording_indicator;
        [FieldOffset(  3)] public byte video_stabilization;
        [FieldOffset(  4)] public byte blank_protected;
        [FieldOffset(  5)] public byte show_mesh;
        [FieldOffset(  6)] public byte shared;
        [FieldOffset(  7)] private byte _reserved_0;
        [FieldOffset(  8)] public float global_opacity;
        [FieldOffset( 12)] public float output_width;
        [FieldOffset( 16)] public float output_height;
        [FieldOffset( 20)] public uint video_stabilization_length;
        [FieldOffset( 24)] public uint hologram_perspective;
        [FieldOffset( 28)] public ushort width;
        [FieldOffset( 30)] public ushort height;
        [FieldOffset( 32)] public byte framerate;
        [FieldOffset( 33)] private byte _reserved_1;
        [FieldOffset( 34), MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)] public string subtype;
        [FieldOffset(162)] private ushort _reserved_2;
        // 164
    };

    [StructLayout(LayoutKind.Explicit, CharSet = CharSet.Unicode)]
    public struct ev_videoformat
    {
        [FieldOffset(0)] public ushort width;
        [FieldOffset(2)] public ushort height;
        [FieldOffset(4)] public byte framerate;
        [FieldOffset(5)] private byte _reserved;
        [FieldOffset(6), MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)] public string subtype;
        // 134
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct vlc_metadata
    {
        [FieldOffset( 0)] public ulong exposure;
        [FieldOffset( 8)] public uint gain;
        [FieldOffset(12)] public uint _reserved;
        // 16
    };

    [StructLayout(LayoutKind.Explicit)]
    public struct pv_metadata
    {
        [FieldOffset(0)] public float fx;
        [FieldOffset(4)] public float fy;
        [FieldOffset(8)] public float cx;
        [FieldOffset(12)] public float cy;
        [FieldOffset(16)] public ulong exposure_time;
        [FieldOffset(24)] public ulong exposure_compensation_0;
        [FieldOffset(32)] public ulong exposure_compensation_1;
        [FieldOffset(40)] public uint lens_position;
        [FieldOffset(44)] public uint focus_state;
        [FieldOffset(48)] public uint iso_speed;
        [FieldOffset(52)] public uint white_balance;
        [FieldOffset(56)] public float analog_gain;
        [FieldOffset(60)] public float digital_gain;
        [FieldOffset(64)] public float r_gain;
        [FieldOffset(68)] public float g_gain;
        [FieldOffset(72)] public float b_gain;
        [FieldOffset(76)] public float extrinsics_mf_tx;
        [FieldOffset(80)] public float extrinsics_mf_ty;
        [FieldOffset(84)] public float extrinsics_mf_tz;
        [FieldOffset(88)] public float extrinsics_mf_rx;
        [FieldOffset(92)] public float extrinsics_mf_ry;
        [FieldOffset(96)] public float extrinsics_mf_rz;
        [FieldOffset(100)] public float extrinsics_mf_rw;
        // 104
    };

    public enum PV_FocusMode
    {
        Auto = 0,
        Single = 1,
        Continuous = 2,
        Manual = 3,
    }

    public enum PV_AutoFocusRange
    {
        FullRange = 0,
        Macro = 1,
        Normal = 2,
    }

    public enum PV_ManualFocusDistance
    {
        Infinity = 0,
        Nearest = 2,
    }

    public enum PV_FocusValue
    {
        Min = 170,
        Max = 10000,
    }

    public enum PV_DriverFallback
    {
        Enable = 0,
        Disable = 1,
    }

    public enum PV_VideoTemporalDenoisingMode
    {
        Off = 0,
        On = 1,
    }

    public enum PV_ColorTemperaturePreset
    {
        Auto = 0,
        Manual = 1,
        Cloudy = 2,
        Daylight = 3,
        Flash = 4,
        Fluorescent = 5,
        Tungsten = 6,
        Candlelight = 7,
    }

    public enum PV_WhiteBalanceValue
    {
        Min = 2300, // 25
        Max = 7500, // 25
    }

    public enum PV_ExposureMode
    {
        Manual = 0,
        Auto = 1,
    }

    public enum PV_ExposureValue
    {
        Min = 1000, // 10
        Max = 660000, // 10
    }

    public enum PV_ExposurePriorityVideo
    {
        Disabled = 0,
        Enabled = 1,
    }

    public enum PV_CaptureSceneMode
    {
        Auto = 0,
        Macro = 2,
        Portrait = 3,
        Sport = 4,
        Snow = 5,
        Night = 6,
        Beach = 7,
        Sunset = 8,
        Candlelight = 9,
        Landscape = 10,
        NightPortrait = 11,
        Backlit = 12,
    }

    public enum PV_IsoSpeedMode
    {
        Manual = 0,
        Auto = 1,
    }

    public enum PV_IsoSpeedValue
    {
        Min = 100,
        Max = 3200,
    }

    public enum PV_BacklightCompensationState
    {
        Disable = 0,
        Enable = 1,
    }

    public enum PV_MediaCaptureOptimization
    {
        Default = 0,
        Quality = 1,
        Latency = 2,
        Power = 3,
        LatencyThenQuality = 4,
        LatencyThenPower = 5,
        PowerAndQuality = 6,
    }

    public enum PV_CaptureUse
    {
        NotSet = 0,
        Photo = 1,
        Video = 2,
    }

    public enum PV_OpticalImageStabilizationMode
    {
        Off = 0,
        On = 1,
    }

    public enum PV_HdrVideoMode
    {
        Off = 0,
        On = 1,
        Auto = 2,
    }

    public enum PV_RegionOfInterestType
    {
        Unknown = 0,
        Face = 1,
    }

    public enum InterfacePriority
    {
        LOWEST = -2,
        BELOW_NORMAL = -1,
        NORMAL = 0,
        ABOVE_NORMAL = 1,
        HIGHEST = 2,
    }

    public enum IMT_Format
    {
        Rgba8 = 30,
        Gray16 = 57,
        Gray8 = 62,
        Bgra8 = 87,
        Nv12 = 103,
        Yuy2 = 107,
    }

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

    public static bool RM_GetResolution(hl2da.SENSOR_ID id, out int w, out int h)
    {
        switch (id)
        {
        case hl2da.SENSOR_ID.RM_VLC_LEFTFRONT:
        case hl2da.SENSOR_ID.RM_VLC_LEFTLEFT:
        case hl2da.SENSOR_ID.RM_VLC_RIGHTFRONT:
        case hl2da.SENSOR_ID.RM_VLC_RIGHTRIGHT:  w = 640; h = 480; break;
        case hl2da.SENSOR_ID.RM_DEPTH_AHAT:      w = 512; h = 512; break;
        case hl2da.SENSOR_ID.RM_DEPTH_LONGTHROW: w = 320; h = 288; break;
        default:                                 w = 0;   h = 0;   return false;
        }

        return true;
    }

    /*
    [DllImport("hl2da")]
    public static extern void Copy(IntPtr source, IntPtr destination, int bytes);

    [DllImport("hl2da")]
    public static extern void DebugMessage(string str);

    [DllImport("hl2da")]
    public static extern void InitializeComponentsOnUI();

    [DllImport("hl2da")]
    public static extern int OverrideWorldCoordinateSystem(IntPtr scs);

    [DllImport("hl2da")]
    public static extern ulong GetUTCOffset(int samples);

    [DllImport("hl2da")]
    public static extern void Initialize(int id, int buffer_size);

    [DllImport("hl2da")]
    public static extern void SetConstantFactorVLC_RM(long factor);

    [DllImport("hl2da")]
    public static extern void BypassDepthLock_RM(int bypass);

    [DllImport("hl2da")]
    public static extern void SetFormat_PV(ref pv_captureformat cf);

    [DllImport("hl2da")]
    public static extern void SetFormat_MC(int raw);

    [DllImport("hl2da")]
    public static extern void SetFormat_EE(int fps_index);

    [DllImport("hl2da")]
    public static extern void SetFormat_EA(ref ea_captureformat cf);

    [DllImport("hl2da")]
    public static extern void SetFormat_EV(ref ev_captureformat cf);

    [DllImport("hl2da")]
    public static extern void SetEnable(int id, int enable);

    [DllImport("hl2da")]
    public static extern int GetByFramestamp(int id, int stamp, ref IntPtr frame, ref ulong timestamp, ref int framestamp);

    [DllImport("hl2da")]
    public static extern int GetByTimestamp(int id, ulong stamp, int time_preference, int tiebreak_right, ref IntPtr frame, ref ulong timestamp, ref int framestamp);

    [DllImport("hl2da")]
    public static extern void Extract(int id, IntPtr frame, ref int valid, IntPtr[] b, int[] l);

    [DllImport("hl2da")]
    public static extern void Release(int id, IntPtr frame);

    [DllImport("hl2da")]
    public static extern void RM_SetEyeSelection(int enable);

    [DllImport("hl2da")]
    public static extern void RM_GetIntrinsics(int id, IntPtr uv2xy, IntPtr mapxy, IntPtr k);

    [DllImport("hl2da")]
    public static extern void RM_GetExtrinsics(int id, IntPtr extrinsics);

    [DllImport("hl2da")]
    public static extern void RM_MapImagePointToCameraUnitPlane(int id, IntPtr image_points, int in_pitch, IntPtr camera_points, int out_pitch, int point_count);

    [DllImport("hl2da")]
    public static extern void RM_MapCameraSpaceToImagePoint(int id, IntPtr camera_points, int in_pitch, IntPtr image_points, int out_pitch, int point_count);

    [DllImport("hl2da")]
    public static extern void PV_SetFocus(uint focusmode, uint autofocusrange, uint distance, uint value, uint disabledriverfallback);

    [DllImport("hl2da")]
    public static extern void PV_SetVideoTemporalDenoising(uint mode);

    [DllImport("hl2da")]
    public static extern void PV_SetWhiteBalance_Preset(uint preset);

    [DllImport("hl2da")]
    public static extern void PV_SetWhiteBalance_Value(uint value);

    [DllImport("hl2da")]
    public static extern void PV_SetExposure(uint setauto, uint value);

    [DllImport("hl2da")]
    public static extern void PV_SetExposurePriorityVideo(uint enabled);

    [DllImport("hl2da")]
    public static extern void PV_SetSceneMode(uint mode);

    [DllImport("hl2da")]
    public static extern void PV_SetIsoSpeed(uint setauto, uint value);

    [DllImport("hl2da")]
    public static extern void PV_SetBacklightCompensation(uint enable);

    [DllImport("hl2da")]
    public static extern void PV_SetDesiredOptimization(uint mode);

    [DllImport("hl2da")]
    public static extern void PV_SetPrimaryUse(uint mode);

    [DllImport("hl2da")]
    public static extern void PV_SetOpticalImageStabilization(uint mode);

    [DllImport("hl2da")]
    public static extern void PV_SetHdrVideo(uint mode);

    [DllImport("hl2da")]
    public static extern void PV_SetRegionsOfInterest(int clear, int set, int auto_exposure, int auto_focus, int bounds_normalized, float x, float y, float w, float h, uint type, uint weight);

    [DllImport("hl2da")]
    public static extern void EX_Request();

    [DllImport("hl2da")]
    public static extern uint EX_Status();

    [DllImport("hl2da")]
    public static extern void EX_SetInterfacePriority(uint id, int priority);

    [DllImport("hl2da")]
    public static extern int EX_GetInterfacePriority(uint id);

    [DllImport("hl2da")]
    public static extern void IMT_ZHTInvalidate(IntPtr depth_in, IntPtr depth_out);

    [DllImport("hl2da")]
    public static extern void IMT_ZLTInvalidate(IntPtr sigma_in, IntPtr depth_in, IntPtr depth_out);

    [DllImport("hl2da")]
    public static extern void IMT_YUV2RGB(IntPtr image, uint stride, uint height, uint format_in, uint format_out, ref IntPtr fc);

    [DllImport("hl2da")]
    public static extern void IMT_Extract(IntPtr fc, ref IntPtr buffer, ref int length);

    [DllImport("hl2da")]
    public static extern void IMT_Release(IntPtr fc);
    */
}
