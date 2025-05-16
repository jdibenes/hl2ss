
using System;
using System.Runtime.InteropServices;

public static partial class hl2ss
{
    public static class svc
    {
        //-----------------------------------------------------------------------------
        // API
        //-----------------------------------------------------------------------------

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_vlc             : hl2ss.ulm.configuration_rm_vlc { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_depth_ahat      : hl2ss.ulm.configuration_rm_depth_ahat { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_depth_longthrow : hl2ss.ulm.configuration_rm_depth_longthrow { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_rm_imu             : hl2ss.ulm.configuration_rm_imu { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_pv                 : hl2ss.ulm.configuration_pv { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_microphone         : hl2ss.ulm.configuration_microphone { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_si                 : hl2ss.ulm.configuration_si { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_eet                : hl2ss.ulm.configuration_eet { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_extended_audio     : hl2ss.ulm.configuration_extended_audio { }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class configuration_extended_depth     : hl2ss.ulm.configuration_extended_depth { }

        [StructLayout(LayoutKind.Sequential, Pack = 4)]
        public class configuration_pv_subsystem       : hl2ss.ulm.configuration_pv_subsystem { }

        public class source : hl2ss.shared.source { public source(string host, ushort port, ulong buffer_size, IntPtr configuration, byte decoded) : base(host, port, buffer_size, configuration, decoded) { } }

        public class ipc_rc  : hl2ss.shared.ipc_rc  { public ipc_rc(string host, ushort port) : base(host, port) { } }
        public class ipc_sm  : hl2ss.shared.ipc_sm  { public ipc_sm(string host, ushort port) : base(host, port) { } }
        public class ipc_su  : hl2ss.shared.ipc_su  { public ipc_su(string host, ushort port) : base(host, port) { } }
        public class ipc_vi  : hl2ss.shared.ipc_vi  { public ipc_vi(string host, ushort port) : base(host, port) { } }
        public class ipc_umq : hl2ss.shared.ipc_umq { public ipc_umq(string host, ushort port) : base(host, port) { } }
        public class ipc_gmq : hl2ss.shared.ipc_gmq { public ipc_gmq(string host, ushort port) : base(host, port) { } }

        public class calibration_view : hl2ss.shared.calibration_view { public calibration_view(string host, ushort port, IntPtr configuration) : base(host, port, configuration) { } }
        public class device_list_view : hl2ss.shared.device_list_view { public device_list_view(string host, ushort port, IntPtr configuration) : base(host, port, configuration) { } }

        public static void initialize()
        {
            hl2ss.shared.initialize();
        }

        public static void cleanup()
        {
            hl2ss.shared.cleanup();
        }

        public static source open_stream<T>(string host, ushort port, ulong buffer_size, T configuration, byte decoded)
        {
            using pointer p = pointer.get(configuration);
            return new source(host, port, buffer_size, p.value, decoded);
        }

        public static source open_stream<T>(string host, ushort port, ulong buffer_size, T configuration, bool decoded)
        {
            return open_stream(host, port, buffer_size, configuration, Convert.ToByte(decoded));
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

        public static void start_subsystem_pv(string host, ushort port, configuration_pv_subsystem configuration)
        {
            hl2ss.shared.start_subsystem_pv(host, port, configuration);
        }

        public static void stop_subsystem_pv(string host, ushort port)
        {
            hl2ss.shared.stop_subsystem_pv(host, port);
        }

        public static calibration_view download_calibration<T>(string host, ushort port, T configuration)
        {
            using pointer p = pointer.get(configuration);
            return new calibration_view(host, port, p.value);
        }

        public static device_list_view download_device_list<T>(string host, ushort port, T configuration)
        {
            using pointer p = pointer.get(configuration);
            return new device_list_view(host, port, p.value);
        }
    }
}
