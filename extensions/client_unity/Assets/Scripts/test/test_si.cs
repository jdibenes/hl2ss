
using System.Runtime.InteropServices;
using UnityEngine;

public class test_si : MonoBehaviour
{
    private hl2ss.shared.source source_si;

    // Start is called before the first frame update
    void Start()
    {
        var configuration = new hl2ss.ulm.configuration_si();

        hl2ss.svc.open_stream(run_once.host_address, hl2ss.stream_port.SPATIAL_INPUT, 300, configuration, true, out source_si);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_si.get_by_index(-1);

        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_si region);
        
        var data = Marshal.PtrToStructure<hl2ss.si_frame>(region.tracking);

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));
        Debug.Log(string.Format("valid {0}", data.valid));
        Debug.Log(string.Format("head_pose position [{0}, {1}, {2}]", data.head_pose.position.x, data.head_pose.position.y, data.head_pose.position.z));
        Debug.Log(string.Format("head_pose up [{0}, {1}, {2}]",       data.head_pose.up.x,       data.head_pose.up.y,       data.head_pose.up.z));
        Debug.Log(string.Format("head_pose forward [{0}, {1}, {2}]",  data.head_pose.forward.x,  data.head_pose.forward.y,  data.head_pose.forward.z));
        Debug.Log(string.Format("eye_ray origin [{0}, {1}, {2}]",     data.eye_ray.origin.x,     data.eye_ray.origin.y,     data.eye_ray.origin.z));
        Debug.Log(string.Format("eye_ray direction [{0}, {1}, {2}]",  data.eye_ray.direction.x,  data.eye_ray.direction.y,  data.eye_ray.direction.z));

        var l_wrist = data.left_hand[hl2ss.si_hand_joint_kind.Wrist];
        var r_wrist = data.right_hand[hl2ss.si_hand_joint_kind.Wrist];

        Debug.Log(string.Format("l_wrist position [{0}, {1}, {2}] orientation [{3}, {4}, {5} {6}] radius {7} accuracy {8}", l_wrist.position.x, l_wrist.position.y, l_wrist.position.z, l_wrist.orientation.x, l_wrist.orientation.y, l_wrist.orientation.z, l_wrist.orientation.w, l_wrist.radius, l_wrist.accuracy));
        Debug.Log(string.Format("r_wrist position [{0}, {1}, {2}] orientation [{3}, {4}, {5} {6}] radius {7} accuracy {8}", r_wrist.position.x, r_wrist.position.y, r_wrist.position.z, r_wrist.orientation.x, r_wrist.orientation.y, r_wrist.orientation.z, r_wrist.orientation.w, r_wrist.radius, r_wrist.accuracy));
    }
}
