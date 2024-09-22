
using System.Runtime.InteropServices;
using UnityEngine;

public class test_si : MonoBehaviour
{
    private hl2ss.svc.source source_si;

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_si configuration);

        source_si = hl2ss.svc.open_stream(host, hl2ss.stream_port.SPATIAL_INPUT, 300, configuration);        
    }

    // Update is called once per frame
    void Update()
    {
        using (var packet = source_si.get_by_index(-1))
        {
            if (packet.status != 0) { return; }

            packet.unpack(out hl2ss.map_si region);

            var data = Marshal.PtrToStructure<hl2ss.si_frame>(region.tracking);

            Debug.Log(string.Format("valid {0}", data.valid));
            Debug.Log(string.Format("head_pose position [{0}, {1}, {2}]", data.head_pose.position.x, data.head_pose.position.y, data.head_pose.position.z));
            Debug.Log(string.Format("head_pose up [{0}, {1}, {2}]", data.head_pose.up.x, data.head_pose.up.y, data.head_pose.up.z));
            Debug.Log(string.Format("head_pose forward [{0}, {1}, {2}]", data.head_pose.forward.x, data.head_pose.forward.y, data.head_pose.forward.z));
            Debug.Log(string.Format("eye_ray origin [{0}, {1}, {2}]", data.eye_ray.origin.x, data.eye_ray.origin.y, data.eye_ray.origin.z));
            Debug.Log(string.Format("eye_ray direction [{0}, {1}, {2}]", data.eye_ray.direction.x, data.eye_ray.direction.y, data.eye_ray.direction.z));

            hl2ss.si_hand_joint left_wrist = data.left_hand[hl2ss.si_hand_joint_kind.Wrist];
            hl2ss.si_hand_joint right_wrist = data.right_hand[hl2ss.si_hand_joint_kind.Wrist];

            Debug.Log(string.Format("left_wrist position [{0}, {1}, {2}]", left_wrist.position.x, left_wrist.position.y, left_wrist.position.z));
            Debug.Log(string.Format("right_wrist position [{0}, {1}, {2}]", right_wrist.position.x, right_wrist.position.y, right_wrist.position.z));
        }
    }

    void OnApplicationQuit()
    {
        if (source_si == null) { return; }

        source_si.Dispose();
    }
}
