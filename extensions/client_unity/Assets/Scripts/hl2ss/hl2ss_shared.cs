
using System;
using System.Runtime.InteropServices;

public static partial class hl2ss
{
    public static class shared
    {
        //-----------------------------------------------------------------------------
        // Response
        //-----------------------------------------------------------------------------

        public static void check_result(int v)
        {
            if (v < 0) { throw new ExternalException("hl2ss::ulm operation error"); }
        }

        public static void check_handle(IntPtr handle)
        {
            if (handle == IntPtr.Zero) { throw new ExternalException("hl2ss::ulm invalid handle"); }
        }

        public class handle : IDisposable
        {
            protected IntPtr m_handle;

            protected handle(IntPtr h)
            {
                m_handle = h;
                check_handle(h);
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
        // Core
        //-----------------------------------------------------------------------------

        public static void initialize()
        {
            check_result(hl2ss.ulm.initialize());
        }

        public static void cleanup()
        {
            check_result(hl2ss.ulm.cleanup());
        }

        //-----------------------------------------------------------------------------
        // Grab
        //-----------------------------------------------------------------------------

        public class packet_view : handle
        {
            protected hl2ss.ulm.packet view;

            protected packet_view(IntPtr h, hl2ss.ulm.packet p) : base(h) 
            {
                view = p;
            }

            protected packet_view(IntPtr source, long frame_stamp, hl2ss.ulm.packet data) : this(hl2ss.ulm.get_by_index(source, frame_stamp, data), data)
            {
            }

            protected packet_view(IntPtr source, ulong timestamp, int time_preference, int tiebreak_right, hl2ss.ulm.packet data) : this(hl2ss.ulm.get_by_timestamp(source, timestamp, time_preference, tiebreak_right, data), data)
            {
            }

            public packet_view(IntPtr source, long frame_stamp) : this(source, frame_stamp, new hl2ss.ulm.packet())
            {
            }

            public packet_view(IntPtr source, ulong timestamp, int time_preference, bool tiebreak_right) : this(source, timestamp, time_preference, Convert.ToInt32(tiebreak_right), new hl2ss.ulm.packet())
            {
            }

            public void unpack(out hl2ss.map_rm_vlc region)
            { 
                region = hl2ss.unpack_rm_vlc(view.payload, view.sz_payload);
            }

            public void unpack(out hl2ss.map_rm_depth_ahat region) 
            { 
                region = hl2ss.unpack_rm_depth_ahat(view.payload, view.sz_payload); 
            }

            public void unpack(out hl2ss.map_rm_depth_longthrow region) 
            { 
                region = hl2ss.unpack_rm_depth_longthrow(view.payload, view.sz_payload);
            }

            public void unpack(out hl2ss.map_rm_imu region) 
            {
                region = hl2ss.unpack_rm_imu(view.payload, view.sz_payload);
            }

            public void unpack(out hl2ss.map_pv region) 
            {
                region = hl2ss.unpack_pv(view.payload, view.sz_payload);
            }

            public void unpack<T>(out hl2ss.map_microphone region) 
            {
                region = hl2ss.unpack_microphone<T>(view.payload, view.sz_payload);
            }

            public void unpack(out hl2ss.map_si region) 
            {
                region = hl2ss.unpack_si(view.payload, view.sz_payload);
            }

            public void unpack(out hl2ss.map_eet region) 
            {
                region = hl2ss.unpack_eet(view.payload, view.sz_payload);
            }

            public void unpack(out hl2ss.map_extended_depth region) 
            {
                region = hl2ss.unpack_extended_depth(view.payload, view.sz_payload);
            }

            public int status { get { return view.status; } }

            public long frame_stamp { get { return view.frame_stamp; } }

            public ulong timestamp { get { return view.timestamp; } }

            public IntPtr payload { get { return view.payload; } }

            public uint sz_payload { get { return view.sz_payload; } }

            public IntPtr pose { get { return view.pose; } }
        }

        //-----------------------------------------------------------------------------
        // Interfaces
        //-----------------------------------------------------------------------------

        public class source : handle
        {
            public source(string host, ushort port, ulong buffer_size, IntPtr configuration, byte decoded) : base(hl2ss.ulm.open_stream(host, port, buffer_size, configuration, decoded))
            {
            }

            public packet_view get_by_index(long frame_stamp)
            {
                return new packet_view(m_handle, frame_stamp);
            }

            public packet_view get_by_timestamp(ulong timestamp, int time_preference, bool tiebreak_right)
            {
                return new packet_view(m_handle, timestamp, time_preference, tiebreak_right);
            }
        }

        public class ipc : handle
        {
            public ipc(string host, ushort port) : base(hl2ss.ulm.open_ipc(host, port))
            {
            }
        }

        //-----------------------------------------------------------------------------
        // Control
        //-----------------------------------------------------------------------------

        public static void start_subsystem_pv(string host, ushort port, hl2ss.ulm.configuration_pv_subsystem c)
        {
            check_result(hl2ss.ulm.start_subsystem_pv(host, port, c));
        }

        public static void stop_subsystem_pv(string host, ushort port)
        {
            check_result(hl2ss.ulm.stop_subsystem_pv(host, port));
        }

        //-----------------------------------------------------------------------------
        // Calibration
        //-----------------------------------------------------------------------------

        public class calibration_view : handle, buffer
        {
            public calibration_view(string host, ushort port, IntPtr configuration) : base(hl2ss.ulm.download_calibration(host, port, configuration, out IntPtr p))
            {
                size = 1;
                data = p;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class device_list_view : handle, buffer
        {
            public device_list_view(string host, ushort port, IntPtr configuration) : base(hl2ss.ulm.download_device_list(host, port, configuration, out ulong s, out IntPtr p))
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
        
        public class rc_rm_map_camera_points_view : handle, buffer
        {
            public rc_rm_map_camera_points_view(IntPtr ipc, ushort port, uint operation, float[] points, uint count) : base(hl2ss.ulm.rc_rm_map_camera_points(ipc, port, operation, points, count, out IntPtr d))
            {
                data = d;
                size = count;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class rc_rm_get_rignode_world_poses_view : handle, buffer
        {
            public rc_rm_get_rignode_world_poses_view(IntPtr ipc, ulong[] timestamps, uint count) : base(hl2ss.ulm.rc_rm_get_rignode_world_poses(ipc, timestamps, count, out IntPtr d))
            {
                data = d;
                size = count;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class ipc_rc : ipc
        {
            public ipc_rc(string host, ushort port) : base(host, port)
            {
            }

            public hl2ss.version ee_get_application_version()
            {
                hl2ss.version version = new hl2ss.version();
                check_result(hl2ss.ulm.rc_ee_get_application_version(m_handle, version));
                return version;
            }

            public ulong ts_get_utc_offset()
            {
                ulong offset;
                check_result(hl2ss.ulm.rc_ts_get_utc_offset(m_handle, out offset));
                return offset;
            }

            public void hs_set_marker_state(uint state)
            {
                check_result(hl2ss.ulm.rc_hs_set_marker_state(m_handle, state));
            }

            public bool pv_get_subsystem_status()
            {
                uint status;
                check_result(hl2ss.ulm.rc_pv_get_subsystem_status(m_handle, out status));
                return status != 0;
            }

            public void pv_wait_for_subsystem(bool status)
            {
                check_result(hl2ss.ulm.rc_pv_wait_for_subsystem(m_handle, Convert.ToByte(status)));
            }

            public void pv_set_focus(uint mode, uint range, uint distance, uint value, uint driver_fallback)
            {
                check_result(hl2ss.ulm.rc_pv_set_focus(m_handle, mode, range, distance, value, driver_fallback));
            }

            public void pv_set_video_temporal_denoising(uint mode)
            {
                check_result(hl2ss.ulm.rc_pv_set_video_temporal_denoising(m_handle, mode));
            }

            public void pv_set_white_balance_preset(uint preset)
            {
                check_result(hl2ss.ulm.rc_pv_set_white_balance_preset(m_handle, preset));
            }

            public void pv_set_white_balance_value(uint value)
            {
                check_result(hl2ss.ulm.rc_pv_set_white_balance_value(m_handle, value));
            }

            public void pv_set_exposure(uint mode, uint value)
            {
                check_result(hl2ss.ulm.rc_pv_set_exposure(m_handle, mode, value));
            }

            public void pv_set_exposure_priority_video(uint enabled)
            {
                check_result(hl2ss.ulm.rc_pv_set_exposure_priority_video(m_handle, enabled));
            }

            public void pv_set_iso_speed(uint mode, uint value)
            {
                check_result(hl2ss.ulm.rc_pv_set_iso_speed(m_handle, mode, value));
            }

            public void pv_set_backlight_compensation(uint state)
            {
                check_result(hl2ss.ulm.rc_pv_set_backlight_compensation(m_handle, state));
            }

            public void pv_set_scene_mode(uint mode)
            {
                check_result(hl2ss.ulm.rc_pv_set_scene_mode(m_handle, mode));
            }

            public void ee_set_flat_mode(bool mode)
            {
                check_result(hl2ss.ulm.rc_ee_set_flat_mode(m_handle, Convert.ToByte(mode)));
            }

            public void rm_set_eye_selection(bool enable)
            {
                check_result(hl2ss.ulm.rc_rm_set_eye_selection(m_handle, Convert.ToByte(enable)));
            }

            public void pv_set_desired_optimization(uint mode)
            {
                check_result(hl2ss.ulm.rc_pv_set_desired_optimization(m_handle, mode));
            }

            public void pv_set_primary_use(uint mode)
            {
                check_result(hl2ss.ulm.rc_pv_set_primary_use(m_handle, mode));
            }

            public void pv_set_optical_image_stabilization(uint mode)
            {
                check_result(hl2ss.ulm.rc_pv_set_optical_image_stabilization(m_handle, mode));
            }

            public void pv_set_hdr_video(uint mode)
            {
                check_result(hl2ss.ulm.rc_pv_set_hdr_video(m_handle, mode));
            }

            public void pv_set_regions_of_interest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, uint type, uint weight, float x, float y, float w, float h)
            {
                check_result(hl2ss.ulm.rc_pv_set_regions_of_interest(m_handle, Convert.ToByte(clear), Convert.ToByte(set), Convert.ToByte(auto_exposure), Convert.ToByte(auto_focus), Convert.ToByte(bounds_normalized), type, weight, x, y, w, h));
            }

            public void ee_set_interface_priority(ushort port, int priority)
            {
                check_result(hl2ss.ulm.rc_ee_set_interface_priority(m_handle, port, priority));
            }

            public void ee_set_quiet_mode(bool mode)
            {
                check_result(hl2ss.ulm.rc_ee_set_quiet_mode(m_handle, Convert.ToByte(mode)));
            }

            public rc_rm_map_camera_points_view rm_map_camera_points(ushort port, uint operation, float[] points, uint count)
            {
                return new rc_rm_map_camera_points_view(m_handle, port, operation, points, count);
            }

            public rc_rm_get_rignode_world_poses_view rm_get_rignode_world_poses(ulong[] timestamps, uint count)
            {
                return new rc_rm_get_rignode_world_poses_view(m_handle, timestamps, count);
            }

            public ulong ts_get_current_time(uint source)
            {
                ulong timestamp;
                check_result(hl2ss.ulm.rc_ts_get_current_time(m_handle, source, out timestamp));
                return timestamp;
            }

            public void si_set_sampling_delay(long delay)
            {
                check_result(hl2ss.ulm.rc_si_set_sampling_delay(m_handle, delay));
            }

            public void ee_set_encoder_buffering(bool enable)
            {
                check_result(hl2ss.ulm.rc_ee_set_encoder_buffering(m_handle, Convert.ToUInt32(enable)));
            }

            public void ee_set_reader_buffering(bool enable)
            {
                check_result(hl2ss.ulm.rc_ee_set_reader_buffering(m_handle, Convert.ToUInt32(enable)));
            }

            public void rm_set_loop_control(ushort port, bool enable)
            {
                check_result(hl2ss.ulm.rc_rm_set_loop_control(m_handle, port, Convert.ToUInt32(enable)));
            }
        }

        //-----------------------------------------------------------------------------
        // Spatial Mapping
        //-----------------------------------------------------------------------------

        public class sm_surface_info_view : handle, buffer
        {
            public sm_surface_info_view(IntPtr ipc) : base(hl2ss.ulm.sm_get_observed_surfaces(ipc, out ulong s, out IntPtr p))
            {
                size = s;
                data = p;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class sm_mesh_collection_view : handle
        {
            protected hl2ss.ulm.sm_mesh[] view;

            public sm_mesh_collection_view(IntPtr ipc, uint count, byte[] data, ulong size) : base(hl2ss.ulm.sm_get_meshes(ipc, count, data, size, out IntPtr meshes_data))
            {
                view = new hl2ss.ulm.sm_mesh[count];
                for (uint i = 0; i < count; ++i) { view[i] = new hl2ss.ulm.sm_mesh(); }
                for (uint i = 0; i < count; ++i) { check_result(hl2ss.ulm.sm_unpack_mesh(meshes_data, i, view[i])); }
            }

            public hl2ss.ulm.sm_mesh[] meshes { get { return view; } }
        }

        public class ipc_sm : ipc
        {
            public ipc_sm(string host, ushort port) : base(host, port)
            {
            }

            public void set_volumes(hl2ss.sm_bounding_volume volumes)
            {
                check_result(hl2ss.ulm.sm_set_volumes(m_handle, volumes.get_count(), volumes.get_data(), volumes.get_size()));
            }

            public sm_surface_info_view get_observed_surfaces()
            {
                return new sm_surface_info_view(m_handle);
            }

            public sm_mesh_collection_view get_meshes(hl2ss.sm_mesh_task tasks)
            {
                return new sm_mesh_collection_view(m_handle, tasks.get_count(), tasks.get_data(), tasks.get_size());
            }
        }

        //-----------------------------------------------------------------------------
        // Scene Understanding
        //-----------------------------------------------------------------------------

        public class su_item_view : hl2ss.ulm.su_item
        {
            public hl2ss.ulm.su_mesh[] unpacked_meshes;
            public hl2ss.ulm.su_mesh[] unpacked_collider_meshes;
        }

        public class su_result_view : handle
        {
            protected hl2ss.ulm.su_result view;
            public su_item_view[] items;

            protected void unpack_meshes(IntPtr meshes_data, ulong count, out hl2ss.ulm.su_mesh[] meshes)
            {
                meshes = new hl2ss.ulm.su_mesh[count];
                for (ulong i = 0; i < count; ++i) { meshes[i] = new hl2ss.ulm.su_mesh(); }
                for (ulong i = 0; i < count; ++i) { check_result(hl2ss.ulm.su_unpack_item_mesh(meshes_data, i, meshes[i])); }
            }

            protected void unpack_item(uint index)
            {
                su_item_view item = items[index];
                unpack_meshes(item.meshes_data, item.meshes_count, out item.unpacked_meshes);
                unpack_meshes(item.collider_meshes_data, item.collider_meshes_count, out item.unpacked_collider_meshes);
            }

            protected su_result_view(IntPtr ipc, hl2ss.ulm.su_task task, hl2ss.ulm.su_result h) : base(hl2ss.ulm.su_query(ipc, task, h))
            {
                view = h;
                if (view.status != 0) { return; }
                items = new su_item_view[view.items_count];
                for (uint i = 0; i < view.items_count; ++i) { items[i] = new su_item_view(); }
                for (uint i = 0; i < view.items_count; ++i) { check_result(hl2ss.ulm.su_unpack_item(view.items_data, i, items[i])); }
                for (uint i = 0; i < view.items_count; ++i) { unpack_item(i); }
            }

            public su_result_view(IntPtr ipc, hl2ss.ulm.su_task task) : this(ipc, task, new hl2ss.ulm.su_result())
            {
            }

            public uint status { get { return view.status; } }

            public matrix_4x4 extrinsics { get { return view.extrinsics; } }

            public matrix_4x4 pose { get { return view.pose; } }

            public ulong items_count { get { return view.items_count; } }

            public IntPtr items_data { get { return view.items_data; } }
        }

        public class ipc_su : ipc
        {
            public ipc_su(string host, ushort port) : base(host, port)
            {
            }

            public su_result_view query(hl2ss.su_task task)
            {
                using pointer p_guid_list = pointer.get(task.guid_list);

                hl2ss.ulm.su_task t = new hl2ss.ulm.su_task();

                t.enable_quads         = Convert.ToByte(task.enable_quads);
                t.enable_meshes        = Convert.ToByte(task.enable_meshes);
                t.enable_only_observed = Convert.ToByte(task.enable_only_observed);
                t.enable_world_mesh    = Convert.ToByte(task.enable_world_mesh);
                t.mesh_lod             = task.mesh_lod;
                t.query_radius         = task.query_radius;
                t.create_mode          = task.create_mode;
                t.kind_flags           = task.kind_flags;
                t.get_orientation      = Convert.ToByte(task.get_orientation);
                t.get_position         = Convert.ToByte(task.get_position);
                t.get_location_matrix  = Convert.ToByte(task.get_location_matrix);
                t.get_quad             = Convert.ToByte(task.get_quad);
                t.get_meshes           = Convert.ToByte(task.get_meshes);
                t.get_collider_meshes  = Convert.ToByte(task.get_collider_meshes);
                t.guid_list_size       = (ulong)task.guid_list.Length;
                t.guid_list_data       = p_guid_list.value;

                return new su_result_view(m_handle, t);
            }
        }

        //-----------------------------------------------------------------------------
        // Voice Input
        //-----------------------------------------------------------------------------

        public class vi_result_view : handle, buffer
        {
            public vi_result_view(IntPtr ipc) : base(hl2ss.ulm.vi_pop(ipc, out ulong s, out IntPtr p))
            {
                size = s;
                data = p;
            }

            public IntPtr data { get; private set; }

            public ulong size { get; private set; }
        }

        public class ipc_vi : ipc
        {
            public ipc_vi(string host, ushort port) : base(host, port)
            {
            }

            public void start(byte[] utf8_array)
            {
                check_result(hl2ss.ulm.vi_start(m_handle, utf8_array));
            }

            public vi_result_view pop()
            {
                return new vi_result_view(m_handle);
            }

            public void stop()
            {
                check_result(hl2ss.ulm.vi_stop(m_handle));
            }
        }

        //-----------------------------------------------------------------------------
        // Unity Message Queue
        //-----------------------------------------------------------------------------

        public class ipc_umq : ipc
        {
            public ipc_umq(string host, ushort port) : base(host, port)
            {
            }

            public void push(byte[] data, uint size)
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

        public class gmq_message_view : handle
        {
            protected hl2ss.ulm.gmq_message view;

            protected gmq_message_view(IntPtr ipc, hl2ss.ulm.gmq_message p) : base(hl2ss.ulm.gmq_pull(ipc, p))
            {
                view = p;
            }

            public gmq_message_view(IntPtr ipc) : this(ipc, new hl2ss.ulm.gmq_message())
            {
            }

            public uint command { get { return view.command; } }

            public uint size { get { return view.size; } }

            public IntPtr data { get { return view.data; } }
        }

        public class ipc_gmq : ipc
        {
            public ipc_gmq(string host, ushort port) : base(host, port)
            {
            }

            public gmq_message_view pull()
            {
                return new gmq_message_view(m_handle);
            }

            public void push(uint[] response, uint count)
            {
                check_result(hl2ss.ulm.gmq_push(m_handle, response, count));
            }
        }
    }
}
