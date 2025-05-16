
using System;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

public class test_ipc_vi : MonoBehaviour
{
    private hl2ss.svc.ipc_vi ipc;

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;
        //string[] commands = new string[] { "cat", "dog", "red", "blue" };
        byte[] commands = Encoding.UTF8.GetBytes("cat\0dog\0red\0blue\0\0");

        hl2ss.svc.open_ipc(host, hl2ss.ipc_port.VOICE_INPUT, out ipc);
        
        ipc.start(commands);
    }

    /*
 * data.ToArray()
 * string[] commands
                List<byte> data = new List<byte>();
        foreach (var s in commands)
        {
            data.AddRange(Encoding.UTF8.GetBytes(s));
            data.Add(0);
        }
        data.Add(0);
*/
    // Update is called once per frame
    void Update()
    {
        using (var result = ipc.pop())
        {
            for (ulong i = 0; i < result.size; ++i)
            {
                var value = Marshal.PtrToStructure<hl2ss.vi_result>(IntPtr.Add(result.data, (int)i * Marshal.SizeOf<hl2ss.vi_result>()));
                Debug.Log(string.Format("VI: {0}, {1}, {2}, {3}, {4}", value.index, value.confidence, value.raw_confidence, value.phrase_start_time, value.phrase_duration));
            }
        }
    }

    void OnApplicationQuit()
    {
        if (ipc == null) { return; }

        ipc.stop();
        ipc.Dispose();
    }
}
