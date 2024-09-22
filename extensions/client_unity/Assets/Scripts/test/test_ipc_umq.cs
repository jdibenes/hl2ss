
using System;
using System.Text;
using UnityEngine;

public class test_ipc_umq : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.open_ipc(host, hl2ss.ipc_port.UNITY_MESSAGE_QUEUE, out hl2ss.svc.ipc_umq ipc);

        var buffer = new hl2ss.umq_command_buffer();
        var response = new uint[1];

        buffer.add(0xFFFFFFFE, Encoding.UTF8.GetBytes("Hello from Unity"));

        ipc.push(buffer.get_data(), buffer.get_size());
        ipc.pull(response, (uint)response.Length);

        Debug.Log(string.Format("Response {0}", response[0]));

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
