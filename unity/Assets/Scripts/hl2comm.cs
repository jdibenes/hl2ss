using AOT;
using System;
using System.Runtime.InteropServices;
using UnityEngine;


public static class hl2comm
{
#if WINDOWS_UWP
    [DllImport("unity_hl2comm")]
    private static extern void InitializeStreamsOnUI(string topic_prefix, string zenoh_config, uint enable);
    [DllImport("unity_hl2comm")]
    private static extern void DebugMessage(string str);
    [DllImport("unity_hl2comm")]
    private static extern void MQ_SO_Push(uint value);
    [DllImport("unity_hl2comm")]
    private static extern void MQ_SI_Pop(out uint command, byte[] data);
    [DllImport("unity_hl2comm")]
    private static extern uint MQ_SI_Peek();
    [DllImport("unity_hl2comm")]
    private static extern void MQ_Restart();
    [DllImport("unity_hl2comm")]
    private static extern void GetLocalIPv4Address(byte[] data, int size);
    [DllImport("unity_hl2comm")]
    private static extern int OverrideWorldCoordinateSystem(IntPtr scs);

#else
    [DllImport("unity_comm")]
    private static extern void InitializeStreamsOnUI(string topic_prefix, string zenoh_config, uint enable);
    [DllImport("unity_comm")]
    private static extern void DebugMessage(string str);
    [DllImport("unity_comm")]
    private static extern void MQ_SO_Push(uint value);
    [DllImport("unity_comm")]
    private static extern void MQ_SI_Pop(out uint command, byte[] data);
    [DllImport("unity_comm")]
    private static extern uint MQ_SI_Peek();
    [DllImport("unity_comm")]
    private static extern void MQ_Restart();

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
        InitializeStreamsOnUI(topic_prefix, zenoh_config, 0U);
    }

    public static void Initialize(string topic_prefix, string zenoh_config, bool enableRM, bool enablePV, bool enableMC, bool enableSI, bool enableRC, bool enableSM, bool enableSU, bool enableVI, bool enableMQ, bool enableEET)
    {
        InitializeStreamsOnUI(topic_prefix, zenoh_config, (enableRM ? 1U : 0U) | (enablePV ? 2U : 0U) | (enableMC ? 4U : 0U) | (enableSI ? 8U : 0U) | (enableRC ? 16U : 0U) | (enableSM ? 32U : 0U) | (enableSU ? 64U : 0U) | (enableVI ? 128U : 0U) | (enableMQ ? 256U : 0U) | (enableEET ? 512U : 0U));
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

    public static bool PullMessage(out uint command, out byte[] data)
    {
        uint size   = MQ_SI_Peek();
        bool status = size != ~0U;
        if (status)
        {
            data = new byte[size];
            MQ_SI_Pop(out command, data);
        }
        else
        {
            data = null;
            command = ~0U;
        }
        return status;
    }

    public static void PushResult(uint value)
    {
        MQ_SO_Push(value);
    }

    public static void AcknowledgeMessage(uint command)
    {
        if (command == ~0U) { MQ_Restart(); }
    }
}
