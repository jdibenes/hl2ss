
using System;

public static partial class hl2ss
{
    public static class svc
    {
        //-----------------------------------------------------------------------------
        // API
        //-----------------------------------------------------------------------------

        public static void initialize()
        {
            hl2ss.shared.initialize();
        }

        public static void cleanup()
        {
            hl2ss.shared.cleanup();
        }

        public static void open_stream<T>(string host, ushort port, ulong buffer_size, T configuration, byte decoded, out hl2ss.shared.source source)
        {
            using pointer p = pointer.get(configuration);
            source = new hl2ss.shared.source(host, port, buffer_size, p.value, decoded);
        }

        public static void open_stream<T>(string host, ushort port, ulong buffer_size, T configuration, bool decoded, out hl2ss.shared.source source)
        {
            open_stream(host, port, buffer_size, configuration, Convert.ToByte(decoded), out source);
        }

        public static void open_ipc(string host, ushort port, out hl2ss.shared.ipc_rc ipc)
        {
            ipc = new hl2ss.shared.ipc_rc(host, port);
        }

        public static void open_ipc(string host, ushort port, out hl2ss.shared.ipc_sm ipc)
        {
            ipc = new hl2ss.shared.ipc_sm(host, port);
        }

        public static void open_ipc(string host, ushort port, out hl2ss.shared.ipc_su ipc)
        {
            ipc = new hl2ss.shared.ipc_su(host, port);
        }

        public static void open_ipc(string host, ushort port, out hl2ss.shared.ipc_vi ipc)
        {
            ipc = new hl2ss.shared.ipc_vi(host, port);
        }

        public static void open_ipc(string host, ushort port, out hl2ss.shared.ipc_umq ipc)
        {
            ipc = new hl2ss.shared.ipc_umq(host, port);
        }

        public static void open_ipc(string host, ushort port, out hl2ss.shared.ipc_gmq ipc)
        {
            ipc = new hl2ss.shared.ipc_gmq(host, port);
        }

        public static void start_subsystem_pv(string host, ushort port, hl2ss.ulm.configuration_pv_subsystem configuration)
        {
            hl2ss.shared.start_subsystem_pv(host, port, configuration);
        }

        public static void stop_subsystem_pv(string host, ushort port)
        {
            hl2ss.shared.stop_subsystem_pv(host, port);
        }

        public static hl2ss.shared.calibration_view download_calibration<T>(string host, ushort port, T configuration)
        {
            using pointer p = pointer.get(configuration);
            return new hl2ss.shared.calibration_view(host, port, p.value);
        }

        public static hl2ss.shared.device_list_view download_device_list<T>(string host, ushort port, T configuration)
        {
            using pointer p = pointer.get(configuration);
            return new hl2ss.shared.device_list_view(host, port, p.value);
        }
    }
}
