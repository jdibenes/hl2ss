
using System.Runtime.InteropServices;
using UnityEngine;

public class test_eet : MonoBehaviour
{
    private hl2ss.shared.source source_eet;

    // Start is called before the first frame update
    void Start()
    {
        var configuration = new hl2ss.ulm.configuration_eet();

        configuration.fps = 90;

        hl2ss.svc.open_stream(run_once.host_address, hl2ss.stream_port.EXTENDED_EYE_TRACKER, 900, configuration, true, out source_eet);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_eet.get_by_index(-1);

        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_eet region);

        var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);
        var data = Marshal.PtrToStructure<hl2ss.eet_frame>(region.tracking);

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));        
        Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose.m_00, pose.m_01, pose.m_02, pose.m_03, pose.m_10, pose.m_11, pose.m_12, pose.m_13, pose.m_20, pose.m_21, pose.m_22, pose.m_23, pose.m_30, pose.m_31, pose.m_32, pose.m_33));
        
        Debug.Log(string.Format("valid {0}", data.valid));
        Debug.Log(string.Format("combined_ray origin [{0}, {1}, {2}]",    data.combined_ray.origin.x,    data.combined_ray.origin.y,    data.combined_ray.origin.z));
        Debug.Log(string.Format("combined_ray direction [{0}, {1}, {2}]", data.combined_ray.direction.x, data.combined_ray.direction.y, data.combined_ray.direction.z));
        Debug.Log(string.Format("left_ray origin [{0}, {1}, {2}]",    data.left_ray.origin.x,   data.left_ray.origin.y,     data.left_ray.origin.z));
        Debug.Log(string.Format("left_ray direction [{0}, {1}, {2}]", data.left_ray.direction.x, data.left_ray.direction.y, data.left_ray.direction.z));
        Debug.Log(string.Format("right_ray origin [{0}, {1}, {2}]",    data.right_ray.origin.x,    data.right_ray.origin.y,    data.right_ray.origin.z));
        Debug.Log(string.Format("right_ray direction [{0}, {1}, {2}]", data.right_ray.direction.x, data.right_ray.direction.y, data.right_ray.direction.z));
    }
}
