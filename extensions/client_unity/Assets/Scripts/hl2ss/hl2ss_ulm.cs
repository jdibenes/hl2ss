
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
            public ulong chunk;
            public byte mode;
            public byte divisor;
            public byte profile;
            public byte level;
            public uint bitrate;
            public long options_size;
            public IntPtr options_data;
            public IntPtr _reserved;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_depth_ahat
        {
            public ulong chunk;
            public byte mode;
            public byte divisor;
            public byte profile_z;
            public byte profile_ab;
            public byte level;
            public byte _reserved_0;
            public ushort _reserved_3;
            public uint bitrate;
            public uint _reserved_1;
            public long options_size;
            public IntPtr options_data;
            public IntPtr _reserved_2;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_depth_longthrow
        {
            public ulong chunk;
            public byte mode;
            public byte divisor;
            public byte png_filter;
            public byte _reserved_0;
            public uint _reserved_1;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_imu
        {
            public ulong chunk;
            public byte mode;
            public byte _reserved_0;
            public ushort _reserved_1;
            public uint _reserved_2;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_pv
        {
            public ulong chunk;
            public byte mode;
            public byte _reserved_0;
            public ushort width;
            public ushort height;
            public byte framerate;
            public byte _reserved_1;
            public byte divisor;
            public byte profile;
            public byte level;
            public byte decoded_format;
            public uint bitrate;
            public long options_size;
            public IntPtr options_data;
            public IntPtr _reserved_2;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_microphone
        {
            public ulong chunk;
            public byte profile;
            public byte level;
            public ushort _reserved_0;
            public uint _reserved_1;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_si
        {
            public ulong chunk;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_eet
        {
            public ulong chunk;
            public byte framerate;
            public byte _reserved_0;
            public ushort _reserved_1;
            public uint _reserved_2;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_extended_audio
        {
            public ulong chunk;
            public uint mixer_mode;
            public float loopback_gain;
            public float microphone_gain;
            public byte profile;
            public byte level;
            public ushort _reserved_0;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_extended_depth
        {
            public ulong chunk;
            public ulong media_index;
            public ulong stride_mask;
            public byte mode;
            public byte divisor;
            public byte profile_z;
            public byte _reserved_0;
            public uint _reserved_1;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 4)]
        public class configuration_pv_subsystem
        {
            public byte enable_mrc;
            public byte hologram_composition;
            public byte recording_indicator;
            public byte video_stabilization;
            public byte blank_protected;
            public byte show_mesh;
            public byte shared;
            public byte _reserved_0;
            public float global_opacity;
            public float output_width;
            public float output_height;
            public uint video_stabilization_length;
            public uint hologram_perspective;
            public uint _reserved_1;
        };

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
            public vector_3 vertex_position_scale;
            public ulong bounds_size;
            public ulong vertex_positions_size;
            public ulong triangle_indices_size;
            public ulong vertex_normals_size;
            public IntPtr pose;
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
            public guid id;
            public int kind;
            public uint _reserved_0;
            public quaternion orientation;
            public vector_3 position;
            public int alignment;
            public vector_2 extents;
            public uint meshes_count;
            public uint collider_meshes_count;
            public IntPtr location;
            public IntPtr meshes;
            public IntPtr collider_meshes;
            public IntPtr _reserved_1;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class su_result_header
        {
            public uint status;
            public uint count;
            public IntPtr extrinsics;
            public IntPtr pose;
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
        // Initialize
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int initialize();

        //-----------------------------------------------------------------------------
        // Interfaces
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr open_stream(string host, ushort port, ulong buffer_size, IntPtr configuration);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr open_ipc(string host, ushort port);

        [DllImport("hl2ss_ulm")]
        public static extern void close_handle(IntPtr h);

        //-----------------------------------------------------------------------------
        // Grab
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr get_by_index(IntPtr source, long frame_stamp, hl2ss.ulm.packet p);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr get_by_timestamp(IntPtr source, ulong timestamp, int time_preference, int tiebreak_right, hl2ss.ulm.packet p);

        [DllImport("hl2ss_ulm")]
        public static extern int get_pv_dimensions(IntPtr source, out ushort width, out ushort height);

        //-----------------------------------------------------------------------------
        // Control
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int start_subsystem_pv(string host, ushort port, configuration_pv_subsystem c);

        [DllImport("hl2ss_ulm")]
        public static extern int stop_subsystem_pv(string host, ushort port);

        //-----------------------------------------------------------------------------
        // Calibration
        //-----------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr download_calibration(string host, ushort port, IntPtr configuration, out IntPtr calibration);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr download_device_list(string host, ushort port, out ulong size, out IntPtr query);

        //------------------------------------------------------------------------------
        // Remote Configuration
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int rc_get_application_version(IntPtr ipc, hl2ss.version version);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_get_utc_offset(IntPtr ipc, out ulong offset);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_hs_marker_state(IntPtr ipc, uint state);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_get_pv_subsystem_status(IntPtr ipc, out uint status);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_wait_for_pv_subsystem(IntPtr ipc, uint status);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_focus(IntPtr ipc, uint mode, uint range, uint distance, uint value, uint driver_fallback);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_video_temporal_denoising(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_white_balance_preset(IntPtr ipc, uint preset);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_white_balance_value(IntPtr ipc, uint value);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_exposure(IntPtr ipc, uint mode, uint value);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_exposure_priority_video(IntPtr ipc, uint enabled);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_iso_speed(IntPtr ipc, uint mode, uint value);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_backlight_compensation(IntPtr ipc, uint state);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_scene_mode(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_flat_mode(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_rm_eye_selection(IntPtr ipc, uint enable);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_desired_optimization(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_primary_use(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_optical_image_stabilization(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_hdr_video(IntPtr ipc, uint mode);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_pv_regions_of_interest(IntPtr ipc, uint clear, uint set, uint auto_exposure, uint auto_focus, uint bounds_normalized, uint type, uint weight, float x, float y, float w, float h);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_interface_priority(IntPtr ipc, ushort port, int priority);

        [DllImport("hl2ss_ulm")]
        public static extern int rc_set_quiet_mode(IntPtr ipc, uint mode);

        //------------------------------------------------------------------------------
        // Spatial Mapping
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern int sm_set_volumes(IntPtr ipc, uint count, byte[] data, ulong size);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr sm_get_observed_surfaces(IntPtr ipc, out ulong size, out IntPtr data);

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr sm_get_meshes(IntPtr ipc, uint count, byte[] data, ulong size);

        [DllImport("hl2ss_ulm")]
        public static extern int sm_unpack_mesh(IntPtr reference, uint index, sm_mesh mesh);

        //------------------------------------------------------------------------------
        // Scene Understanding
        //------------------------------------------------------------------------------

        [DllImport("hl2ss_ulm")]
        public static extern IntPtr su_query(IntPtr ipc, su_task task, su_result_header header);

        [DllImport("hl2ss_ulm")]
        public static extern int su_unpack_item(IntPtr reference, uint index, su_item item);

        [DllImport("hl2ss_ulm")]
        public static extern int su_unpack_item_mesh(IntPtr meshes, uint index, su_mesh mesh);

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
        public static extern int umq_push(IntPtr ipc, byte[] data, ulong size);

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
