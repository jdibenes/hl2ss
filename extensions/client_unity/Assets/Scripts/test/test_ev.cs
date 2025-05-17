
using System.Text;
using System.Runtime.InteropServices;
using UnityEngine;

public class test_ev : MonoBehaviour
{
    public GameObject quad_pv;
    public float group_index = 0.0f;
    public float source_index = 2.0f;
    public float profile_index = 4.0f;

    private string host;
    private hl2ss.shared.source source_ev;
    private int pv_frame_size;
    private Texture2D tex_pv;
    TextureFormat texture_format;
    int bpp;

    // Start is called before the first frame update
    void Start()
    {
        host = run_once.host_address;

        var configuration = new hl2ss.ulm.configuration_pv();

        configuration.width = 1280;
        configuration.height = 720;
        configuration.framerate = 30;

        var decoded_format = hl2ss.pv_decoded_format.RGB;
        texture_format = TextureFormat.RGB24;
        bpp = 3;

        var configuration_subsystem = new hl2ss.ulm.configuration_pv_subsystem();

        configuration_subsystem.global_opacity = group_index;
        configuration_subsystem.output_width   = source_index;
        configuration_subsystem.output_height  = profile_index;

        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.EXTENDED_VIDEO, configuration_subsystem);

        using var device_list_handle = hl2ss.svc.download_device_list(host, hl2ss.stream_port.EXTENDED_VIDEO, configuration);
        var string_bytes = new byte[device_list_handle.size];
        Marshal.Copy(device_list_handle.data, string_bytes, 0, (int)device_list_handle.size);
        Debug.Log(Encoding.Unicode.GetString(string_bytes));

        hl2ss.svc.open_stream(host, hl2ss.stream_port.EXTENDED_VIDEO, 300, configuration, decoded_format, out source_ev);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_ev.get_by_index(-1);

        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_pv region);

        var metadata = Marshal.PtrToStructure<hl2ss.pv_metadata>(region.metadata);
        var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));
        Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose.m_00, pose.m_01, pose.m_02, pose.m_03, pose.m_10, pose.m_11, pose.m_12, pose.m_13, pose.m_20, pose.m_21, pose.m_22, pose.m_23, pose.m_30, pose.m_31, pose.m_32, pose.m_33));
        
        Debug.Log(string.Format("resolution {0}x{1}", metadata.width, metadata.height));

        if (!tex_pv)
        {
            var width = metadata.width;
            var height = metadata.height;

            pv_frame_size = width * height * bpp;
            tex_pv = new Texture2D(width, height, texture_format, false);
            quad_pv.GetComponent<Renderer>().material.mainTexture = tex_pv;
        }

        tex_pv.LoadRawTextureData(region.image, pv_frame_size);
        tex_pv.Apply();
    }

    void OnApplicationQuit()
    {
        if (source_ev == null) { return; }

        source_ev.Dispose();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.EXTENDED_VIDEO);
    }
}
