
using System;
using System.Runtime.InteropServices;

public static partial class hl2ss
{
    public class svc
    {
        //-----------------------------------------------------------------------------
        // Handle
        //-----------------------------------------------------------------------------

        public class handle
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

            public void destroy()
            {
                if (m_handle == IntPtr.Zero) { return; }
                hl2ss.ulm.close_handle(m_handle);
                m_handle = IntPtr.Zero;
            }

            ~handle()
            {
                destroy();
            }
        }

        // BLOCK

        //-----------------------------------------------------------------------------
        // Stream
        //-----------------------------------------------------------------------------

        public class packet : handle
        {
            protected hl2ss.ulm.packet data;

            protected packet(IntPtr h, hl2ss.ulm.packet data) : base(h)
            {
                this.data = data;
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

            public void unpack(out hl2ss.map_pv region)
            {
                region = hl2ss.unpack_pv(payload, sz_payload);
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
        }

        // BLOCK

        //-----------------------------------------------------------------------------
        // API
        //-----------------------------------------------------------------------------

        public static void initialize()
        {
            handle.check_result(hl2ss.ulm.initialize());
        }

        public static hl2ss.ulm.configuration_rm_vlc create_configuration_rm_vlc()
        {
            hl2ss.ulm.configuration_rm_vlc c = new hl2ss.ulm.configuration_rm_vlc();

            c.chunk = hl2ss.chunk_size.RM_VLC;
            c.mode = hl2ss.stream_mode.MODE_1;
            c.divisor = 1;
            c.profile = hl2ss.video_profile.H265_MAIN;
            c.level = hl2ss.h26x_level.DEFAULT;
            c.bitrate = 0;
            c.options_data = IntPtr.Zero;
            c.options_size = -1;

            return c;
        }

        public static hl2ss.ulm.configuration_rm_depth_ahat create_configuration_rm_depth_ahat()
        {
            hl2ss.ulm.configuration_rm_depth_ahat c = new hl2ss.ulm.configuration_rm_depth_ahat();

            c.chunk = hl2ss.chunk_size.RM_DEPTH_AHAT;
            c.mode = hl2ss.stream_mode.MODE_1;
            c.divisor = 1;
            c.profile_z = hl2ss.depth_profile.SAME;
            c.profile_ab = hl2ss.video_profile.H265_MAIN;
            c.level = hl2ss.h26x_level.DEFAULT;
            c.bitrate = 0;
            c.options_data = IntPtr.Zero;
            c.options_size = -1;

            return c;
        }

        public static hl2ss.ulm.configuration_rm_depth_longthrow create_configuration_rm_depth_longthrow()
        {
            hl2ss.ulm.configuration_rm_depth_longthrow c = new hl2ss.ulm.configuration_rm_depth_longthrow();

            c.chunk = hl2ss.chunk_size.RM_DEPTH_LONGTHROW;
            c.mode = hl2ss.stream_mode.MODE_1;
            c.divisor = 1;
            c.png_filter = hl2ss.png_filter_mode.PAETH;

            return c;
        }

        public static hl2ss.ulm.configuration_rm_imu create_configuration_rm_imu()
        {
            hl2ss.ulm.configuration_rm_imu c = new hl2ss.ulm.configuration_rm_imu();

            c.chunk = hl2ss.chunk_size.RM_IMU;
            c.mode = hl2ss.stream_mode.MODE_1;

            return c;
        }

        public static hl2ss.ulm.configuration_pv create_configuration_pv()
        {
            hl2ss.ulm.configuration_pv c = new hl2ss.ulm.configuration_pv();

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

            return c;
        }

        public static hl2ss.ulm.configuration_microphone create_configuration_microphone()
        {
            hl2ss.ulm.configuration_microphone c = new hl2ss.ulm.configuration_microphone();

            c.chunk = hl2ss.chunk_size.MICROPHONE;
            c.profile = hl2ss.audio_profile.AAC_24000;
            c.level = hl2ss.aac_level.L2;

            return c;
        }

        public static hl2ss.ulm.configuration_si create_configuration_si()
        {
            hl2ss.ulm.configuration_si c = new hl2ss.ulm.configuration_si();

            c.chunk = hl2ss.chunk_size.SPATIAL_INPUT;

            return c;
        }

        public static hl2ss.ulm.configuration_eet create_configuration_eet()
        {
            hl2ss.ulm.configuration_eet c = new hl2ss.ulm.configuration_eet();

            c.chunk = hl2ss.chunk_size.EXTENDED_EYE_TRACKER;
            c.framerate = hl2ss.eet_framerate.FPS_30;

            return c;
        }

        public static hl2ss.ulm.configuration_extended_audio create_configuration_extended_audio()
        {
            hl2ss.ulm.configuration_extended_audio c = new hl2ss.ulm.configuration_extended_audio();

            c.chunk = hl2ss.chunk_size.EXTENDED_AUDIO;
            c.mixer_mode = hl2ss.mixer_mode.BOTH;
            c.loopback_gain = 1.0f;
            c.microphone_gain = 1.0f;
            c.profile = hl2ss.audio_profile.AAC_24000;
            c.level = hl2ss.aac_level.L2;

            return c;
        }

        public static source open_stream<T>(string host, ushort port, ulong buffer_size, T configuration)
        {
            GCHandle h = GCHandle.Alloc(configuration, GCHandleType.Pinned);
            source s = new source(host, port, buffer_size, h.AddrOfPinnedObject());
            h.Free();
            return s;
        }

        // BLOCK

        public static void start_subsystem_pv(string host, ushort port, byte enable_mrc = 0, byte hologram_composition = 1, byte recording_indicator = 0, byte video_stabilization = 0, byte blank_protected = 0, byte show_mesh = 0, byte shared = 0, float global_opacity = 0.9f, float output_width = 0.0f, float output_height = 0.0f, uint video_stabilization_length = 0, uint hologram_perspective = hl2ss.hologram_perspective.PV)
        {
            handle.check_result(hl2ss.ulm.start_subsystem_pv(host, port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective));
        }

        public static void stop_subsystem_pv(string host, ushort port)
        {
            handle.check_result(hl2ss.ulm.stop_subsystem_pv(host, port));
        }




}
}
