
using System.Runtime.InteropServices;
using UnityEngine;

public class test_eet : MonoBehaviour
{
    private hl2ss.svc.source source_eet;

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_eet configuration);

        configuration.framerate = hl2ss.eet_framerate.FPS_90;

        source_eet = hl2ss.svc.open_stream(host, hl2ss.stream_port.EXTENDED_EYE_TRACKER, 900, configuration);
    }

    // Update is called once per frame
    void Update()
    {
        using (var packet = source_eet.get_by_index(-1))
        {
            if (packet.status != 0) { return; }

            packet.unpack(out hl2ss.map_eet region);

            var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);
            var data = Marshal.PtrToStructure<hl2ss.eet_frame>(region.tracking);

            Debug.Log(string.Format("valid {0}", data.valid));

            Debug.Log(string.Format("combined_ray origin [{0}, {1}, {2}]", data.combined_ray.origin.x, data.combined_ray.origin.y, data.combined_ray.origin.z));
            Debug.Log(string.Format("combined_ray direction [{0}, {1}, {2}]", data.combined_ray.direction.x, data.combined_ray.direction.y, data.combined_ray.direction.z));

            Debug.Log(string.Format("left_ray origin [{0}, {1}, {2}]", data.left_ray.origin.x, data.left_ray.origin.y, data.left_ray.origin.z));
            Debug.Log(string.Format("left_ray direction [{0}, {1}, {2}]", data.left_ray.direction.x, data.left_ray.direction.y, data.left_ray.direction.z));

            Debug.Log(string.Format("right_ray origin [{0}, {1}, {2}]", data.right_ray.origin.x, data.right_ray.origin.y, data.right_ray.origin.z));
            Debug.Log(string.Format("right_ray direction [{0}, {1}, {2}]", data.right_ray.direction.x, data.right_ray.direction.y, data.right_ray.direction.z));
        }
    }

    void OnApplicationQuit()
    {
        if (source_eet == null) { return; }

        source_eet.Dispose();
    }
}
