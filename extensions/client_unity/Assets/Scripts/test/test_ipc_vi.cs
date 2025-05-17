
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

public class test_ipc_vi : MonoBehaviour
{
    private hl2ss.shared.ipc_vi ipc;
    private string[] commands = new string[] { "cat", "dog", "red", "blue" };

    // Start is called before the first frame update
    void Start()
    {
        var data = new List<byte>();
        foreach (var s in commands)
        {
            data.AddRange(Encoding.UTF8.GetBytes(s));
            data.Add(0);
        }
        data.Add(0);

        hl2ss.svc.open_ipc(run_once.host_address, hl2ss.ipc_port.VOICE_INPUT, out ipc);
        
        ipc.start(data.ToArray());
    }

    // Update is called once per frame
    void Update()
    {
        using var result = ipc.pop();
        for (ulong i = 0; i < result.size; ++i)
        {
            var value = Marshal.PtrToStructure<hl2ss.vi_result>(IntPtr.Add(result.data, (int)i * Marshal.SizeOf<hl2ss.vi_result>()));
            Debug.Log(string.Format("VI: index {0} confidence {1} raw_confidence {2} phrase_start_time {3} phrase_duration {4} command {5}", value.index, value.confidence, value.raw_confidence, value.phrase_start_time, value.phrase_duration, commands[value.index]));
        }
    }

    void OnApplicationQuit()
    {
        if (ipc == null) { return; }

        ipc.stop();
        ipc.Dispose();
    }
}
