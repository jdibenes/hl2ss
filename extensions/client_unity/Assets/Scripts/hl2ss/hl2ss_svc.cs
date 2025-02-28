
using System;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;

public static partial class hl2ss
{
    public static class svc
    {
        //-----------------------------------------------------------------------------
        // Handle
        //-----------------------------------------------------------------------------

        public class handle : IDisposable
        {
            protected IntPtr m_handle;

            protected static void check_result(IntPtr handle)
            {
                if (handle == IntPtr.Zero) { throw new ExternalException("ULM invalid handle"); }
            }

            protected handle(IntPtr h)
            {
                m_handle = h;
                check_result(h);
            }

            public static void check_result(int v)
            {
                if (v < 0) { throw new ExternalException("ULM operation error"); }
            }

            protected virtual void Dispose(bool disposing)
            {
                if (m_handle == IntPtr.Zero) { return; }
                hl2ss.ulm.close_handle(m_handle);
                m_handle = IntPtr.Zero;
            }

            public void Dispose()
            {
                Dispose(true);
                GC.SuppressFinalize(this);
            }

            ~handle()
            {
                Dispose(false);
            }
        }

        public interface buffer
        {
            public ulong size { get; }

            public IntPtr data { get; }
        }

        //-----------------------------------------------------------------------------
        // Stream
        //-----------------------------------------------------------------------------

        public class packet : handle
        {
            protected hl2ss.ulm.packet data;

            protected packet(IntPtr h, hl2ss.ulm.packet p) : base(h)
            {
                data = p;
            }

            protected packet(IntPtr source, long frame_stamp, hl2ss.ulm.packet data) : this(hl2ss.ulm.get_by_index(source, frame_stamp, data), data)
            {
            }

            protected packet(IntPtr source, ulong timestamp, int time_preference, int tiebreak_right, hl2ss.ulm.packet data) : this(hl2ss.ulm.get_by_timestamp(source, timestamp, time_preference, tiebreak_right, data), data)
            {
            }

            public packet(IntPtr source, long frame_stamp) : this(source, frame_stamp, new hl2ss.ulm.packet())
            {
            }

            public packet(IntPtr source, ulong timestamp, int time_preference, int tiebreak_right) : this(source, timestamp, time_preference, tiebreak_right, new hl2ss.ulm.packet())
            {
            }

            public long frame_stamp { get { return data.frame_stamp; } }

            public ulong timestamp { get { return data.timestamp; } }

            public uint sz_payload { get { return data.sz_payload; } }

            public int status { get { return data.status; } }

            public IntPtr payload { get { return data.payload; } }

            public IntPtr pose { get { return data.pose; } }

            public void unpack(out hl2ss.map_rm_vlc region)
            {
                region = hl2ss.unpack_rm_vlc(payload);
            }

            public void unpack(out hl2ss.map_rm_depth_ahat region)
            {
                region = hl2ss.unpack_rm_depth_ahat(payload);
            }

            public void unpack(out hl2ss.map_rm_depth_longthrow region)
            {
                region = hl2ss.unpack_rm_depth_longthrow(payload);
            }

            public void unpack(out hl2ss.map_rm_imu region)
            {
                region = hl2ss.unpack_rm_imu(payload);
            }

            public void unpack(out hl2ss.map_pv region)
            {
                region = hl2ss.unpack_pv(payload, sz_payload);
            }

            public void unpack(out hl2ss.map_microphone_aac region)
            {
                region = hl2ss.unpack_microphone_aac(payload);
            }

            public void unpack(out hl2ss.map_microphone_raw region)
            {
                region = hl2ss.unpack_microphone_raw(payload);
            }

            public void unpack(out hl2ss.map_microphone_array region)
            {
                region = hl2ss.unpack_microphone_array(payload);
            }

            public void unpack(out hl2ss.map_si region)
            {
                region = hl2ss.unpack_si(payload);
            }

            public void unpack(out hl2ss.map_eet region)
            {
                region = hl2ss.unpack_eet(payload);
            }

            public void unpack(out hl2ss.map_extended_audio_aac region)
            {
                region = hl2ss.unpack_extended_audio_aac(payload);
            }

            public void unpack(out hl2ss.map_extended_audio_raw region)
            {
                region = hl2ss.unpack_extended_audio_raw(payload);
            }

            public void unpack(out hl2ss.map_extended_depth region)
            {
                region = hl2ss.unpack_extended_depth(payload, sz_payload);
            }
        }

        public class source : handle
        {
            public source(string host, ushort port, ulong buffer_size, IntPtr configuration) : base(hl2ss.ulm.open_stream(host, port, buffer_size, configuration))
            {
            }

            public packet get_by_index(long frame_stamp)
            {
                return new packet(m_handle, frame_stamp);
            }

            public packet get_by_timestamp(ulong timestamp, int time_preference, int tiebreak_right)
            {
                return new packet(m_handle, timestamp, time_preference, tiebreak_right);
            }

            public void get_pv_dimensions(out ushort width, out ushort height)
            {
                check_result(hl2ss.ulm.get_pv_dimensions(m_handle, out width, out height));
            }
        }

        public class calibration : handle, buffer
        {
            public calibration(string host, ushort port, IntPtr configuration) : base(hl2ss.ulm.download_calibration(host, port, configuration, out IntPtr p))
            {
                size = 1;
                data = p;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class device_list : handle, buffer
        {
            public device_list(string host, ushort port) : base(hl2ss.ulm.download_device_list(host, port, out ulong s, out IntPtr p))
            {
                size = s;
                data = p;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        //-----------------------------------------------------------------------------
        // Remote Configuration
        //-----------------------------------------------------------------------------

        public class ipc_rc : handle
        {
            public ipc_rc(string host, ushort port) : base(hl2ss.ulm.open_ipc(host, port))
            {
            }

            public hl2ss.version get_application_version()
            {
                hl2ss.version version = new hl2ss.version();
                check_result(hl2ss.ulm.rc_get_application_version(m_handle, version));
                return version;
            }

            public ulong get_utc_offset()
            {
                ulong offset;
                check_result(hl2ss.ulm.rc_get_utc_offset(m_handle, out offset));
                return offset;
            }

            public void set_hs_marker_state(uint state)
            {
                check_result(hl2ss.ulm.rc_set_hs_marker_state(m_handle, state));
            }

            public bool get_pv_subsystem_status()
            {
                uint status;
                check_result(hl2ss.ulm.rc_get_pv_subsystem_status(m_handle, out status));
                return status != 0;
            }

            public void wait_for_pv_subsystem(bool status)
            {
                check_result(hl2ss.ulm.rc_wait_for_pv_subsystem(m_handle, Convert.ToByte(status)));
            }

            public void set_pv_focus(uint mode, uint range, uint distance, uint value, uint driver_fallback)
            {
                check_result(hl2ss.ulm.rc_set_pv_focus(m_handle, mode, range, distance, value, driver_fallback));
            }

            public void set_pv_video_temporal_denoising(uint mode)
            {
                check_result(hl2ss.ulm.rc_set_pv_video_temporal_denoising(m_handle, mode));
            }

            public void set_pv_white_balance_preset(uint preset)
            {
                check_result(hl2ss.ulm.rc_set_pv_white_balance_preset(m_handle, preset));
            }

            public void set_pv_white_balance_value(uint value)
            {
                check_result(hl2ss.ulm.rc_set_pv_white_balance_value(m_handle, value));
            }

            public void set_pv_exposure(uint mode, uint value)
            {
                check_result(hl2ss.ulm.rc_set_pv_exposure(m_handle, mode, value));
            }

            public void set_pv_exposure_priority_video(uint enabled)
            {
                check_result(hl2ss.ulm.rc_set_pv_exposure_priority_video(m_handle, enabled));
            }

            public void set_pv_iso_speed(uint mode, uint value)
            {
                check_result(hl2ss.ulm.rc_set_pv_iso_speed(m_handle, mode, value));
            }

            public void set_pv_backlight_compensation(uint state)
            {
                check_result(hl2ss.ulm.rc_set_pv_backlight_compensation(m_handle, state));
            }

            public void set_pv_scene_mode(uint mode)
            {
                check_result(hl2ss.ulm.rc_set_pv_scene_mode(m_handle, mode));
            }

            public void set_flat_mode(bool mode)
            {
                check_result(hl2ss.ulm.rc_set_flat_mode(m_handle, Convert.ToByte(mode)));
            }

            public void set_rm_eye_selection(bool enable)
            {
                check_result(hl2ss.ulm.rc_set_rm_eye_selection(m_handle, Convert.ToByte(enable)));
            }

            public void set_pv_desired_optimization(uint mode)
            {
                check_result(hl2ss.ulm.rc_set_pv_desired_optimization(m_handle, mode));
            }

            public void set_pv_primary_use(uint mode)
            {
                check_result(hl2ss.ulm.rc_set_pv_primary_use(m_handle, mode));
            }

            public void set_pv_optical_image_stabilization(uint mode)
            {
                check_result(hl2ss.ulm.rc_set_pv_optical_image_stabilization(m_handle, mode));
            }

            public void set_pv_hdr_video(uint mode)
            {
                check_result(hl2ss.ulm.rc_set_pv_hdr_video(m_handle, mode));
            }

            public void set_pv_regions_of_interest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, uint type, uint weight, float x, float y, float w, float h)
            {
                check_result(hl2ss.ulm.rc_set_pv_regions_of_interest(m_handle, Convert.ToByte(clear), Convert.ToByte(set), Convert.ToByte(auto_exposure), Convert.ToByte(auto_focus), Convert.ToByte(bounds_normalized), type, weight, x, y, w, h));
            }

            public void set_interface_priority(ushort port, int priority)
            {
                check_result(hl2ss.ulm.rc_set_interface_priority(m_handle, port, priority));
            }

            public void set_quiet_mode(bool mode)
            {
                check_result(hl2ss.ulm.rc_set_quiet_mode(m_handle, Convert.ToByte(mode)));
            }
        }

        //-----------------------------------------------------------------------------
        // Spatial Mapping
        //-----------------------------------------------------------------------------

        public class sm_surface_info_collection : handle, buffer
        {
            public sm_surface_info_collection(IntPtr ipc) : base(hl2ss.ulm.sm_get_observed_surfaces(ipc, out ulong s, out IntPtr p))
            {
                size = s;
                data = p;                
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class sm_mesh_collection : handle
        {
            public hl2ss.ulm.sm_mesh[] meshes;

            public sm_mesh_collection(IntPtr ipc, uint count, byte[] data, ulong size) : base(hl2ss.ulm.sm_get_meshes(ipc, count, data, size))
            {
                meshes = new hl2ss.ulm.sm_mesh[count];
                for (uint i = 0; i < count; ++i)
                {
                    meshes[i] = new hl2ss.ulm.sm_mesh();
                    check_result(hl2ss.ulm.sm_unpack_mesh(m_handle, i, meshes[i]));
                }
            }
        }

        public class ipc_sm : handle
        {
            public ipc_sm(string host, ushort port) : base(hl2ss.ulm.open_ipc(host, port))
            {
            }

            public void set_volumes(hl2ss.sm_bounding_volume volumes)
            {
                check_result(hl2ss.ulm.sm_set_volumes(m_handle, volumes.get_count(), volumes.get_data(), volumes.get_size()));
            }

            public sm_surface_info_collection get_observed_surfaces()
            {
                return new sm_surface_info_collection(m_handle);
            }

            public sm_mesh_collection get_meshes(hl2ss.sm_mesh_task tasks)
            {
                return new sm_mesh_collection(m_handle, tasks.get_count(), tasks.get_data(), tasks.get_size());
            }
        }

        //-----------------------------------------------------------------------------
        // Scene Understanding
        //-----------------------------------------------------------------------------

        public class su_item
        {
            public hl2ss.ulm.su_item content;
            public hl2ss.ulm.su_mesh[] unpacked_meshes;
            public hl2ss.ulm.su_mesh[] unpacked_collider_meshes;
        }

        public class su_result : handle
        {
            public hl2ss.ulm.su_result_header header;
            public su_item[] items;

            protected void unpack_meshes(IntPtr meshes, uint count, out hl2ss.ulm.su_mesh[] o)
            {
                o = new hl2ss.ulm.su_mesh[count];
                for (uint i = 0; i < count; ++i)
                {
                    o[i] = new hl2ss.ulm.su_mesh();
                    check_result(hl2ss.ulm.su_unpack_item_mesh(meshes, i, o[i]));
                }
            }

            protected void unpack_item(uint index)
            {
                su_item item = items[index];
                unpack_meshes(item.content.meshes, item.content.meshes_count, out item.unpacked_meshes);
                unpack_meshes(item.content.collider_meshes, item.content.collider_meshes_count, out item.unpacked_collider_meshes);
            }

            protected su_result(IntPtr ipc, hl2ss.ulm.su_task task, hl2ss.ulm.su_result_header h) : base(hl2ss.ulm.su_query(ipc, task, h))
            {
                header = h;
                if (header.status != 0) { return; }
                items = new su_item[header.count];
                for (uint i = 0; i < header.count; ++i)
                {
                    items[i] = new su_item();
                    items[i].content = new hl2ss.ulm.su_item();
                    check_result(hl2ss.ulm.su_unpack_item(m_handle, i, items[i].content));
                    unpack_item(i);
                }
            }

            public su_result(IntPtr ipc, hl2ss.ulm.su_task task) : this(ipc, task, new hl2ss.ulm.su_result_header())
            {
            }
        }

        public class ipc_su : handle
        {
            public ipc_su(string host, ushort port) : base(hl2ss.ulm.open_ipc(host, port))
            {
            }

            public su_result query(hl2ss.su_task task)
            {
                hl2ss.ulm.su_task t = new hl2ss.ulm.su_task();

                t.enable_quads = Convert.ToByte(task.enable_quads);
                t.enable_meshes = Convert.ToByte(task.enable_meshes);
                t.enable_only_observed = Convert.ToByte(task.enable_only_observed);
                t.enable_world_mesh = Convert.ToByte(task.enable_world_mesh);
                t.mesh_lod = task.mesh_lod;
                t.query_radius = task.query_radius;
                t.create_mode = task.create_mode;
                t.kind_flags = task.kind_flags;
                t.get_orientation = Convert.ToByte(task.get_orientation);
                t.get_position = Convert.ToByte(task.get_position);
                t.get_location_matrix = Convert.ToByte(task.get_location_matrix);
                t.get_quad = Convert.ToByte(task.get_quad);
                t.get_meshes = Convert.ToByte(task.get_meshes);
                t.get_collider_meshes = Convert.ToByte(task.get_collider_meshes);

                using (pointer p = pointer.get(task.guid_list))
                {
                t.guid_list_size = (ulong)task.guid_list.Length;
                t.guid_list_data = p.value;

                return new su_result(m_handle, t);
                }
            }
        }

        //-----------------------------------------------------------------------------
        // Voice Input
        //-----------------------------------------------------------------------------

        public class vi_result : handle, buffer
        {
            public vi_result(IntPtr ipc) : base(hl2ss.ulm.vi_pop(ipc, out ulong s, out IntPtr p))
            {
                size = s;
                data = p;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class ipc_vi : handle
        {
            public ipc_vi(string host, ushort port) : base(hl2ss.ulm.open_ipc(host, port))
            {
            }

            public void start(string[] commands)
            {
                List<byte> data = new List<byte>();
                foreach (var s in commands)
                {
                    data.AddRange(Encoding.UTF8.GetBytes(s));
                    data.Add(0);
                }
                data.Add(0);
                check_result(hl2ss.ulm.vi_start(m_handle, data.ToArray()));
            }

            public vi_result pop()
            {
                return new vi_result(m_handle);
            }

            public void stop()
            {
                check_result(hl2ss.ulm.vi_stop(m_handle));
            }
        }

        //-----------------------------------------------------------------------------
        // Unity Message Queue
        //-----------------------------------------------------------------------------

        public class ipc_umq : handle
        {
            public ipc_umq(string host, ushort port) : base(hl2ss.ulm.open_ipc(host, port))
            {
            }

            public void push(byte[] data, ulong size)
            {
                check_result(hl2ss.ulm.umq_push(m_handle, data, size));
            }

            public void pull(uint[] data, uint count)
            {
                check_result(hl2ss.ulm.umq_pull(m_handle, data, count));
            }
        }

        //-----------------------------------------------------------------------------
        // Guest Message Queue
        //-----------------------------------------------------------------------------

        public class gmq_message : handle
        {
            hl2ss.ulm.gmq_message message;

            protected gmq_message(IntPtr ipc, hl2ss.ulm.gmq_message p) : base(hl2ss.ulm.gmq_pull(ipc, p))
            {
                message = p;
            }

            public gmq_message(IntPtr ipc) : this(ipc, new hl2ss.ulm.gmq_message())
            {
            }

            public uint command { get { return message.command; } }

            public uint size { get { return message.size; } }

            public IntPtr data { get { return message.data; } }
        }

        public class ipc_gmq : handle
        {
            public ipc_gmq(string host, ushort port) : base(hl2ss.ulm.open_ipc(host, port))
            {
            }

            public gmq_message pull()
            {
                return new gmq_message(m_handle);
            }

            public void push(uint[] response, uint count)
            {
                check_result(hl2ss.ulm.gmq_push(m_handle, response, count));
            }
        }

        //-----------------------------------------------------------------------------
        // API
        //-----------------------------------------------------------------------------

        public static void initialize()
        {
            handle.check_result(hl2ss.ulm.initialize());
        }

        public static void create_configuration(out hl2ss.ulm.configuration_rm_vlc c)
        {
            c = new hl2ss.ulm.configuration_rm_vlc();

            c.chunk = hl2ss.chunk_size.RM_VLC;
            c.mode = hl2ss.stream_mode.MODE_1;
            c.divisor = 1;
            c.profile = hl2ss.video_profile.H265_MAIN;
            c.level = hl2ss.h26x_level.DEFAULT;
            c.bitrate = 0;
            c.options_data = IntPtr.Zero;
            c.options_size = -1;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_rm_depth_ahat c)
        {
            c = new hl2ss.ulm.configuration_rm_depth_ahat();

            c.chunk = hl2ss.chunk_size.RM_DEPTH_AHAT;
            c.mode = hl2ss.stream_mode.MODE_1;
            c.divisor = 1;
            c.profile_z = hl2ss.depth_profile.SAME;
            c.profile_ab = hl2ss.video_profile.H265_MAIN;
            c.level = hl2ss.h26x_level.DEFAULT;
            c.bitrate = 0;
            c.options_data = IntPtr.Zero;
            c.options_size = -1;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_rm_depth_longthrow c)
        {
            c = new hl2ss.ulm.configuration_rm_depth_longthrow();

            c.chunk = hl2ss.chunk_size.RM_DEPTH_LONGTHROW;
            c.mode = hl2ss.stream_mode.MODE_1;
            c.divisor = 1;
            c.png_filter = hl2ss.png_filter_mode.PAETH;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_rm_imu c)
        {
            c = new hl2ss.ulm.configuration_rm_imu();

            c.chunk = hl2ss.chunk_size.RM_IMU;
            c.mode = hl2ss.stream_mode.MODE_1;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_pv c)
        {
            c = new hl2ss.ulm.configuration_pv();

            c.width = 1920;
            c.height = 1080;
            c.framerate = 30;
            c.chunk = hl2ss.chunk_size.PERSONAL_VIDEO;
            c.mode = hl2ss.stream_mode.MODE_1;
            c.divisor = 1;
            c.profile = hl2ss.video_profile.H265_MAIN;
            c.level = hl2ss.h26x_level.DEFAULT;
            c.bitrate = 0;
            c.options_data = IntPtr.Zero;
            c.options_size = -1;
            c.decoded_format = hl2ss.pv_decoded_format.BGR;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_microphone c)
        {
            c = new hl2ss.ulm.configuration_microphone();

            c.chunk = hl2ss.chunk_size.MICROPHONE;
            c.profile = hl2ss.audio_profile.AAC_24000;
            c.level = hl2ss.aac_level.L2;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_si c)
        {
            c = new hl2ss.ulm.configuration_si();

            c.chunk = hl2ss.chunk_size.SPATIAL_INPUT;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_eet c)
        {
            c = new hl2ss.ulm.configuration_eet();

            c.chunk = hl2ss.chunk_size.EXTENDED_EYE_TRACKER;
            c.framerate = hl2ss.eet_framerate.FPS_30;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_extended_audio c)
        {
            c = new hl2ss.ulm.configuration_extended_audio();

            c.chunk = hl2ss.chunk_size.EXTENDED_AUDIO;
            c.mixer_mode = hl2ss.mixer_mode.BOTH;
            c.loopback_gain = 1.0f;
            c.microphone_gain = 1.0f;
            c.profile = hl2ss.audio_profile.AAC_24000;
            c.level = hl2ss.aac_level.L2;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_extended_depth c)
{
            c = new hl2ss.ulm.configuration_extended_depth();

            c.chunk = hl2ss.chunk_size.EXTENDED_DEPTH;
            c.media_index = 0xFFFFFFFF;
            c.stride_mask = 0x3F;
            c.mode = hl2ss.stream_mode.MODE_0;
            c.divisor = 1;
            c.profile_z = hl2ss.depth_profile.ZDEPTH;
        }

        public static void create_configuration(out hl2ss.ulm.configuration_pv_subsystem c)
        {
            c = new hl2ss.ulm.configuration_pv_subsystem();

            c.enable_mrc = 0;
            c.hologram_composition = 1;
            c.recording_indicator = 0;
            c.video_stabilization = 0;
            c.blank_protected = 0;
            c.show_mesh = 0;
            c.shared = 0;
            c.global_opacity = 0.9f;
            c.output_width = 0.0f;
            c.output_height = 0.0f;
            c.video_stabilization_length = 0;
            c.hologram_perspective = hl2ss.hologram_perspective.PV;
        }

        public static source open_stream<T>(string host, ushort port, ulong buffer_size, T configuration)
        {
            using (pointer p = pointer.get(configuration)) { return new source(host, port, buffer_size, p.value); }
        }

        public static void open_ipc(string host, ushort port, out ipc_rc ipc)
        {
            ipc = new ipc_rc(host, port);
        }

        public static void open_ipc(string host, ushort port, out ipc_sm ipc)
        {
            ipc = new ipc_sm(host, port);
        }

        public static void open_ipc(string host, ushort port, out ipc_su ipc)
        {
            ipc = new ipc_su(host, port);
        }

        public static void open_ipc(string host, ushort port, out ipc_vi ipc)
        {
            ipc = new ipc_vi(host, port);
        }

        public static void open_ipc(string host, ushort port, out ipc_umq ipc)
        {
            ipc = new ipc_umq(host, port);
        }

        public static void open_ipc(string host, ushort port, out ipc_gmq ipc)
        {
            ipc = new ipc_gmq(host, port);
        }

        public static void start_subsystem_pv(string host, ushort port, hl2ss.ulm.configuration_pv_subsystem c)
        {
            handle.check_result(hl2ss.ulm.start_subsystem_pv(host, port, c));
        }

        public static void stop_subsystem_pv(string host, ushort port)
        {
            handle.check_result(hl2ss.ulm.stop_subsystem_pv(host, port));
        }

        public static calibration download_calibration<T>(string host, ushort port, T configuration)
        {
            using (pointer p = pointer.get(configuration)) { return new calibration(host, port, p.value); }
        }

        public static device_list download_device_list(string host, ushort port)
        {
            return new device_list(host, port);
        }
    }
}
