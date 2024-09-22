using System.Text;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

public class test_ipc_gmq : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.open_ipc(host, hl2ss.ipc_port.GUEST_MESSAGE_QUEUE, out hl2ss.svc.ipc_gmq ipc);

        uint[] response = new uint[1] { 1 };

        using (var msg = ipc.pull())
        {
            Debug.Log(string.Format("command id: {0} size: {1}", msg.command, msg.size));
            if ((msg.command == 0xFFFFFFFE) && (msg.size > 0))
            {
                byte[] data = new byte[msg.size];
                Marshal.Copy(msg.data, data, 0, (int)msg.size);
                Debug.Log(Encoding.UTF8.GetString(data));
            }

            ipc.push(response, (uint)response.Length);
        }

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
