using AOT;
using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Events;

namespace tcn
{

 
    // wrap some essential types from Zenoh for communication (taken from zenoh-csharp branch of sanri)

    public enum ZEncodingPrefix : int // z_encoding_prefix_t
    {
        Empty = 0,
        AppOctetStream = 1,
        AppCustom = 2,
        TextPlain = 3,
        AppProperties = 4,
        AppJson = 5,
        AppSql = 6,
        AppInteger = 7,
        AppFloat = 8,
        AppXml = 9,
        AppXhtmlXml = 10,
        AppXWwwFormUrlencoded = 11,
        TextJson = 12,
        TextHtml = 13,
        TextXml = 14,
        TextCss = 15,
        TextCsv = 16,
        TextJavascript = 17,
        ImageJpeg = 18,
        ImagePng = 19,
        ImageGif = 20,
    }

    public enum ZSampleKind : int // z_sample_kind_t
    {
        Put = 0,
        Delete = 1,
    }

    public enum ZTarget : int // z_target_t
    {
        BestMatching,
        All,
        None,
        AllComplete,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Zid
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        internal byte[] id;

        public string ToStr()
        {
            return hl2comm.IdBytesToStr(id);
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ZBytes // z_bytes_t, z_owned_bytes_t
    {
        public ulong len;
        public IntPtr start;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ZTimestamp // z_timestamp_t
    {
        public UInt64 time;
        public Zid id;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ZString // z_owned_string_t, z_string_t
    {
        internal IntPtr start;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ZEncoding // z_encoding_t
    {
        internal ZEncodingPrefix prefix;
        internal ZBytes suffix;

        public String PrefixToString()
        {
            return prefix.ToString();
        }

        public static ZEncoding New(ZEncodingPrefix prefix)
        {
            return FnZEncoding(prefix, IntPtr.Zero);
        }

        [DllImport(hl2comm.ZenohDllName, EntryPoint = "z_encoding", CallingConvention = CallingConvention.Cdecl)]
        internal static extern ZEncoding FnZEncoding(ZEncodingPrefix prefix, IntPtr suffix);
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Period // z_period_t
    {
        public uint origin;
        public uint period;
        public uint duration;

        public Period(uint origin, uint period, uint duration)
        {
            this.origin = origin;
            this.period = period;
            this.duration = duration;
        }
    }

    public class KeyExpr : IDisposable
    {
        [StructLayout(LayoutKind.Sequential)]
        public struct NativeType // z_keyexpr_t 
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            private UInt64[] _align;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            private UIntPtr[] _padding;

            internal string GetStr()
            {
                IntPtr ptr = ZKeyexprToString(this);
                string output = Marshal.PtrToStringAnsi(ptr);
                // should be free using libc free()
                Marshal.FreeHGlobal(ptr);
                return output;
            }
        }

        internal NativeType native;
        private IntPtr _key_buf;
        private bool _disposed;

        public void Dispose() => Dispose(true);

        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            Marshal.FreeHGlobal(this._key_buf);
            _disposed = true;
        }

        internal KeyExpr(NativeType native, IntPtr keyBuf)
        {
            this.native = native;
            this._key_buf = keyBuf;
            this._disposed = false;
        }

        public static KeyExpr FromString(string name)
        {
            IntPtr p = Marshal.StringToHGlobalAnsi(name);
            NativeType keyexpr = ZKeyexpr(p);
            //Marshal.FreeHGlobal(p);
            return new KeyExpr(keyexpr, p);
        }

        public string GetStr()
        {
            return native.GetStr();
        }

        [DllImport(hl2comm.ZenohDllName, EntryPoint = "z_keyexpr", CallingConvention = CallingConvention.Cdecl)]
        internal static extern NativeType ZKeyexpr(IntPtr name);

        [DllImport(hl2comm.ZenohDllName, EntryPoint = "z_keyexpr_to_string", CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ZKeyexprToString(NativeType keyexpr);
    }

    public class Sample
    {
        //[StructLayout(LayoutKind.Explicit)]
        //[FieldOffset(0)]
        [StructLayout(LayoutKind.Sequential)]
        public struct NativeType // z_owned_sample_t
        {
            internal KeyExpr.NativeType keyexpr;
            internal ZBytes payload;
            internal ZEncoding encoding;
            internal IntPtr _zc_buf;
            internal ZSampleKind kind;
            internal ZTimestamp timestamp;
        }

        public string Key { get; }
        public byte[] Value { get; }
        public ZEncoding Encoding { get; }

        internal Sample(NativeType sample)
        {
            Key = sample.keyexpr.GetStr();
            UnityEngine.Debug.Log("New Sample with key: " + Key);
            Value = ZTypes.ZBytesToBytesArray(sample.payload);
            Encoding = sample.encoding;
        }

        public string ValueToString()
        {
            string result = System.Text.Encoding.UTF8.GetString(Value, 0, Value.Length);
            return result;
        }
    }

    internal static class ZTypes
    {
        internal static byte[] ZBytesToBytesArray(ZBytes zb)
        {
            UnityEngine.Debug.Log("ZBTBA: len " + (int)zb.len);
            byte[] managedArray = new byte[(int)zb.len];
            System.Runtime.InteropServices.Marshal.Copy(zb.start, managedArray, 0, (int)zb.len);
            return managedArray;
        }

        internal static string ZBytesToString(ZBytes zb)
        {
            byte[] managedArray = new byte[(int)zb.len];
            System.Runtime.InteropServices.Marshal.Copy(zb.start, managedArray, 0, (int)zb.len);
            string result = System.Text.Encoding.UTF8.GetString(managedArray, 0, (int)zb.len);
            return result;
        }

        internal static string ZStringToString(ZString zs)
        {
            return Marshal.PtrToStringAnsi(zs.start);
        }

        [DllImport(hl2comm.ZenohDllName, EntryPoint = "z_string_free", CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ZStringFree(ref ZString s);

        [DllImport(hl2comm.ZenohDllName, EntryPoint = "z_string_check", CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool ZStringCheck(ref ZString s);

        [DllImport(hl2comm.ZenohDllName, EntryPoint = "z_bytes_free", CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ZBytesFree(ref ZString s);
    }

    // wrap unity_hl2comm/comm

    public static class hl2comm
    {

        // setup the dll interface for the plugin
#if WINDOWS_UWP
    public const string DllName = "unity_hl2comm";
#else
        public const string DllName = "unity_comm";
#endif
        public const string ZenohDllName = "zenohc";

        private static UnityEvent<bool> haveSessionEvent = new UnityEvent<bool>();


        [DllImport(hl2comm.DllName)]
        private static extern void InitializeStreamsOnUI(string topic_prefix, string zenoh_config, uint enable);

        [DllImport(hl2comm.DllName)]
        private static extern void TeardownStreamsOnUI();

        [DllImport(hl2comm.DllName)]
        private static extern void DebugMessage(string str);

        [DllImport(hl2comm.DllName)]
        private static extern bool ZSendMessage(string keyexpr, IntPtr buffer, ulong buffer_len, ZEncodingPrefix encoding, bool block);


        // unity_comm may also implement these functions later if it makes sense without HL2 ...
#if WINDOWS_UWP
    private static extern void GetLocalIPv4Address(byte[] data, int size);
    [DllImport("unity_hl2comm")]
    private static extern int OverrideWorldCoordinateSystem(IntPtr scs);

#else
        private static void GetLocalIPv4Address(byte[] data, int size)
        {
        }

        private static int OverrideWorldCoordinateSystem(IntPtr scs)
        {
            return 1;
        }
#endif

        public static void Initialize(string topic_prefix, string zenoh_config)
        {
            UnityEngine.Debug.Log("Initialize Unity Comm Plugin.");
            InitializeStreamsOnUI(topic_prefix, zenoh_config, 0U);
            haveSessionEvent.Invoke(true);
        }

        public static void Initialize(string topic_prefix, string zenoh_config, bool enableRM, bool enablePV, bool enableMC, bool enableSI, bool enableRC, bool enableSM, bool enableSU, bool enableVI, bool enableMQ, bool enableEET)
        {
            UnityEngine.Debug.Log("Initialize Unity Comm Plugin.");
            InitializeStreamsOnUI(topic_prefix, zenoh_config, (enableRM ? 1U : 0U) | (enablePV ? 2U : 0U) | (enableMC ? 4U : 0U) | (enableSI ? 8U : 0U) | (enableRC ? 16U : 0U) | (enableSM ? 32U : 0U) | (enableSU ? 64U : 0U) | (enableVI ? 128U : 0U) | (enableMQ ? 256U : 0U) | (enableEET ? 512U : 0U));
            haveSessionEvent.Invoke(true);
        }

        public static bool PutStr(string key, string s)
        {
            byte[] data = Encoding.UTF8.GetBytes(s);
            return _Put(key, data, ZEncodingPrefix.TextPlain);
        }

        public static bool PutJson(string key, string s)
        {
            byte[] data = Encoding.UTF8.GetBytes(s);
            return _Put(key, data, ZEncodingPrefix.AppJson);
        }

        public static bool PutInt(string key, Int64 value)
        {
            byte[] data = Encoding.UTF8.GetBytes(value.ToString());
            return _Put(key, data, ZEncodingPrefix.AppInteger);
        }

        public static bool PutFloat(string key, double value)
        {
            string s = value.ToString();
            byte[] data = Encoding.UTF8.GetBytes(s);
            return _Put(key, data, ZEncodingPrefix.AppFloat);
        }

        public static bool _Put(string key, byte[] value, ZEncodingPrefix encoding)
        {
            IntPtr v = Marshal.AllocHGlobal(value.Length);
            Marshal.Copy(value, 0, v, value.Length);
            bool r = ZSendMessage(key, v, (ulong)value.Length, encoding, true);
            Marshal.FreeHGlobal(v);
            return r;
        }

        public static void Teardown()
        {
            UnityEngine.Debug.Log("Teardown Unity Comm Plugin.");
            haveSessionEvent.Invoke(false);
            TeardownStreamsOnUI();
        }
        public static void RegisterForSessionEvent(UnityAction<bool> action)
        {
            haveSessionEvent.AddListener(action); // register action to receive the event callback
        }

        public static void UnregisterSessionEvent(UnityAction<bool> action)
        {
            haveSessionEvent.RemoveListener(action); // unregister to stop receiving the event callback
        }

        public static void Print(string str)
        {
            DebugMessage(str);
        }

        public static string GetIPAddress()
        {
            byte[] ipaddress = new byte[16 * 2];
            GetLocalIPv4Address(ipaddress, ipaddress.Length);
            return System.Text.Encoding.Unicode.GetString(ipaddress);
        }

        public static bool UpdateCoordinateSystem()
        {
            var scs = Microsoft.MixedReality.OpenXR.PerceptionInterop.GetSceneCoordinateSystem(Pose.identity);
            if (scs == null) { return false; }
            var unk = Marshal.GetIUnknownForObject(scs);
            bool ret = OverrideWorldCoordinateSystem(unk) != 0;
            Marshal.Release(unk);
            return ret;
        }

        public static string IdBytesToStr(byte[] buf)
        {
            StringBuilder str = new StringBuilder();
            for (int i = buf.Length - 1; i >= 0; i--)
            {
                str.Append($"{buf[i]:X2}");
            }

            return str.ToString();
        }

        internal const int RoutersNum = 128;
        internal const int PeersNum = 256;
        internal const int IdLength = 16;

    }



}
