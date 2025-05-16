
using System;
using System.Runtime.InteropServices;

public static partial class hl2ss
{
    public static class ulm
    {
        //-----------------------------------------------------------------------------
        // Adapters
        //-----------------------------------------------------------------------------

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_vlc
        {
            public ulong chunk = hl2ss.chunk_size.RM_VLC;
            public byte mode = hl2ss.stream_mode.MODE_1;
            public byte divisor = 1;
            public byte profile = hl2ss.video_profile.H265_MAIN;
            public byte level = hl2ss.h26x_level.DEFAULT;
            public uint bitrate = 0;
            public long options_size = -1;
            public IntPtr options_data = IntPtr.Zero;
            public IntPtr _reserved; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_depth_ahat
        {
            public ulong chunk = hl2ss.chunk_size.RM_DEPTH_AHAT;
            public byte mode = hl2ss.stream_mode.MODE_1;
            public byte divisor = 1;
            public byte profile_z = hl2ss.depth_profile.SAME;
            public byte profile_ab = hl2ss.video_profile.H265_MAIN;
            public byte level = hl2ss.h26x_level.DEFAULT;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public byte[] _reserved_0; //
            public uint bitrate = 0;
            public uint _reserved_1; //
            public long options_size = -1;
            public IntPtr options_data = IntPtr.Zero;
            public IntPtr _reserved_2; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_depth_longthrow
        {
            public ulong chunk = hl2ss.chunk_size.RM_DEPTH_LONGTHROW;
            public byte mode = hl2ss.stream_mode.MODE_1;
            public byte divisor = 1;
            public byte png_filter = hl2ss.png_filter_mode.PAETH;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)]
            public byte[] _reserved; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_imu
        {
            public ulong chunk = hl2ss.chunk_size.RM_IMU;
            public byte mode = hl2ss.stream_mode.MODE_1;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public byte[] _reserved; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_pv
        {
            public ulong chunk = hl2ss.chunk_size.PERSONAL_VIDEO;
            public byte mode = hl2ss.stream_mode.MODE_1;
            public byte _reserved_0; //
            public ushort width = 1920;
            public ushort height = 1080;
            public byte framerate = 30;
            public byte divisor = 1;
            public byte profile = hl2ss.video_profile.H265_MAIN;
            public byte level = hl2ss.h26x_level.DEFAULT;
            public ushort _reserved_1; //
            public uint bitrate = 0;
            public long options_size = -1;
            public IntPtr options_data = IntPtr.Zero;
            public IntPtr _reserved_2; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_microphone
        {
            public ulong chunk = hl2ss.chunk_size.MICROPHONE;
            public byte profile = hl2ss.audio_profile.AAC_24000;
            public byte level = hl2ss.aac_level.L2;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public byte[] _reserved; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_si
        {
            public ulong chunk = hl2ss.chunk_size.SPATIAL_INPUT;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_eet
        {
            public ulong chunk = hl2ss.chunk_size.EXTENDED_EYE_TRACKER;
            public byte fps = 30;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public byte[] _reserved; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_extended_audio
        {
            public ulong chunk = hl2ss.chunk_size.EXTENDED_AUDIO;
            public uint mixer_mode = hl2ss.mixer_mode.BOTH;
            public float loopback_gain = 1.0f;
            public float microphone_gain = 1.0f;
            public byte profile = hl2ss.audio_profile.AAC_24000;
            public byte level = hl2ss.aac_level.L2;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            public byte[] _reserved; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_extended_depth
        {
            public ulong chunk = hl2ss.chunk_size.EXTENDED_DEPTH;
            public ulong media_index = 0xFFFFFFFF;
            public ulong stride_mask = 0x3F;
            public byte mode = hl2ss.stream_mode.MODE_1;
            public byte divisor = 1;
            public byte profile_z = hl2ss.depth_profile.ZDEPTH;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)]
            public byte[] _reserved; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 4)]
        public class configuration_pv_subsystem
        {
            public byte enable_mrc = Convert.ToByte(false);
            public byte hologram_composition = Convert.ToByte(true);
            public byte recording_indicator = Convert.ToByte(false);
            public byte video_stabilization = Convert.ToByte(false);
            public byte blank_protected = Convert.ToByte(false);
            public byte show_mesh = Convert.ToByte(false);
            public byte shared = Convert.ToByte(false);
            public byte _reserved_0; //
            public float global_opacity = 0.9f;
            public float output_width = 0.0f;
            public float output_height = 0.0f;
            public uint video_stabilization_length = 0;
            public uint hologram_perspective = hl2ss.hologram_perspective.PV;
            public uint _reserved_1; //
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class packet
        {
            public long frame_stamp;
            public ulong timestamp;
            public uint sz_payload;
            public int status;
            public IntPtr payload;
            public IntPtr pose;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class sm_mesh
        {
            public uint status;
            public hl2ss.vector_3 vertex_position_scale;
            public ulong bounds_size;
            public ulong vertex_positions_size;
            public ulong triangle_indices_size;
            public ulong vertex_normals_size;
            public hl2ss.matrix_4x4 pose;
            public IntPtr bounds_data;
            public IntPtr vertex_positions_data;
            public IntPtr triangle_indices_data;
            public IntPtr vertex_normals_data;
            public IntPtr _reserved;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class su_mesh
        {
            public ulong vertex_positions_size;
            public ulong triangle_indices_size;
            public IntPtr vertex_positions_data;
            public IntPtr triangle_indices_data;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class su_item
        {
            public hl2ss.guid id;
            public int kind;
            public uint _reserved_0;
            public hl2ss.quaternion orientation;
            public hl2ss.vector_3 position;
            public int alignment;
            public hl2ss.vector_2 extents;
            public ulong meshes_count;
            public ulong collider_meshes_count;
            public hl2ss.matrix_4x4 location;
            public IntPtr meshes_data;
            public IntPtr collider_meshes_data;
            public IntPtr _reserved_1;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class su_result
        {
            public uint status;
            public uint _reserved;
            public ulong items_count;
            public hl2ss.matrix_4x4 extrinsics;
            public hl2ss.matrix_4x4 pose;
            public IntPtr items_data;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class su_task
        {
            public byte enable_quads;
            public byte enable_meshes;
            public byte enable_only_observed;
            public byte enable_world_mesh;
            public uint mesh_lod;
            public float query_radius;
            public byte create_mode;
            public byte kind_flags;
            public byte get_orientation;
            public byte get_position;
            public byte get_location_matrix;
            public byte get_quad;
            public byte get_meshes;
            public byte get_collider_meshes;
            public uint _reserved_0;
            public ulong guid_list_size;
            public IntPtr guid_list_data;
            public IntPtr _reserved_1;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class gmq_message
        {
            public uint command;
            public uint size;
            public IntPtr data;
            public IntPtr _reserved;
        }

        //-----------------------------------------------------------------------------
        // Core
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int initialize();

        [DllImport("hl2ss_ulm")]
        public static extern int cleanup();

        [DllImport("hl2ss_ulm")]
        public static extern void close_handle(IntPtr h);

        //-----------------------------------------------------------------------------
        // Interfaces
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr open_stream(string host, ushort port, ulong buffer_size, IntPtr configuration, byte decoded);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr open_ipc(string host, ushort port);

        //-----------------------------------------------------------------------------
        // Grab
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr get_by_index(IntPtr source, long frame_stamp, hl2ss.ulm.packet p);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr get_by_timestamp(IntPtr source, ulong timestamp, int time_preference, int tiebreak_right, hl2ss.ulm.packet p);

        //-----------------------------------------------------------------------------
        // Control
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int start_subsystem_pv(string host, ushort port, hl2ss.ulm.configuration_pv_subsystem c);

        [DllImport("hl2ss_ulm")]
        public static extern int stop_subsystem_pv(string host, ushort port);

        //-----------------------------------------------------------------------------
        // Calibration
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr download_calibration(string host, ushort port, IntPtr configuration, out IntPtr calibration);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr download_device_list(string host, ushort port, IntPtr configuration, out ulong size, out IntPtr query);

        //------------------------------------------------------------------------------
        // Remote Configuration
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ee_get_application_version(IntPtr ipc, hl2ss.version version);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ts_get_utc_offset(IntPtr ipc, out ulong offset);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_hs_set_marker_state(IntPtr ipc, uint state);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_get_subsystem_status(IntPtr ipc, out uint status);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_wait_for_subsystem(IntPtr ipc, uint status);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_focus(IntPtr ipc, uint mode, uint range, uint distance, uint value, uint driver_fallback);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_video_temporal_denoising(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_white_balance_preset(IntPtr ipc, uint preset);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_white_balance_value(IntPtr ipc, uint value);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_exposure(IntPtr ipc, uint mode, uint value);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_exposure_priority_video(IntPtr ipc, uint enabled);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_iso_speed(IntPtr ipc, uint mode, uint value);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_backlight_compensation(IntPtr ipc, uint state);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_scene_mode(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ee_set_flat_mode(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_rm_set_eye_selection(IntPtr ipc, uint enable);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_desired_optimization(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_primary_use(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_optical_image_stabilization(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_hdr_video(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_pv_set_regions_of_interest(IntPtr ipc, uint clear, uint set, uint auto_exposure, uint auto_focus, uint bounds_normalized, uint type, uint weight, float x, float y, float w, float h);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ee_set_interface_priority(IntPtr ipc, ushort port, int priority);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ee_set_quiet_mode(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr rc_rm_map_camera_points(IntPtr ipc, ushort port, uint operation, float[] points, uint count, out IntPtr result);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr rc_rm_get_rignode_world_poses(IntPtr ipc, ulong[] timestamps, uint count, out IntPtr poses);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ts_get_current_time(IntPtr ipc, uint source, out ulong timestamp);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_si_set_sampling_delay(IntPtr ipc, long delay);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ee_set_encoder_buffering(IntPtr ipc, uint enable);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_ee_set_reader_buffering(IntPtr ipc, uint enable);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_rm_set_loop_control(IntPtr ipc, ushort port, uint enable);

        //------------------------------------------------------------------------------
        // Spatial Mapping
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int sm_set_volumes(IntPtr ipc, uint count, byte[] data, ulong size);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr sm_get_observed_surfaces(IntPtr ipc, out ulong size, out IntPtr data);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr sm_get_meshes(IntPtr ipc, uint count, byte[] data, ulong size, out IntPtr meshes_data);

        [DllImport("hl2ss_ulm")]
        public static extern int sm_unpack_mesh(IntPtr meshes_data, ulong index, hl2ss.ulm.sm_mesh mesh);

        //------------------------------------------------------------------------------
        // Scene Understanding
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr su_query(IntPtr ipc, hl2ss.ulm.su_task task, hl2ss.ulm.su_result header);

        [DllImport("hl2ss_ulm")]
        public static extern int su_unpack_item(IntPtr items_data, ulong index, hl2ss.ulm.su_item item);

        [DllImport("hl2ss_ulm")]
        public static extern int su_unpack_item_mesh(IntPtr meshes_data, ulong index, hl2ss.ulm.su_mesh mesh);

        //------------------------------------------------------------------------------
        // Voice Input
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int vi_start(IntPtr ipc, byte[] utf8_array);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr vi_pop(IntPtr ipc, out ulong size, out IntPtr data);

        [DllImport("hl2ss_ulm")]
        public static extern int vi_stop(IntPtr ipc);

        //------------------------------------------------------------------------------
        // Unity Message Queue
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int umq_push(IntPtr ipc, byte[] data, uint size);

        [DllImport("hl2ss_ulm")]
        public static extern int umq_pull(IntPtr ipc, uint[] data, uint count);

        //-----------------------------------------------------------------------------
        // Guest Message Queue
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr gmq_pull(IntPtr ipc, gmq_message result);

        [DllImport("hl2ss_ulm")]
        public static extern int gmq_push(IntPtr ipc, uint[] response, uint count);
    }
}
