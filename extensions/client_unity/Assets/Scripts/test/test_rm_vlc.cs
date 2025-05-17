
using System.Runtime.InteropServices;
using UnityEngine;

public class test_rm_vlc : MonoBehaviour
{
    public GameObject quad_rm_vlc;
    public ushort port = hl2ss.stream_port.RM_VLC_LEFTFRONT;
    public Shader shader_vlc;

    private Texture2D tex_vlc;
    private RenderTexture texr_vlc;
    private Material mat_vlc;

    private hl2ss.shared.source source_rm_vlc;

    // Start is called before the first frame update
    void Start()
    {
        var host = run_once.host_address;

        var configuration = new hl2ss.ulm.configuration_rm_vlc();

        using var calibration_handle = hl2ss.svc.download_calibration(host, port, configuration);
        var calibration = Marshal.PtrToStructure<hl2ss.calibration_rm_vlc>(calibration_handle.data);

        Debug.Log(string.Format("intrinsics fx {0} fy {1} cx {2} cy {3}", calibration.intrinsics[0], calibration.intrinsics[1], calibration.intrinsics[2], calibration.intrinsics[3]));
        Debug.Log(string.Format("extrinsics [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", calibration.extrinsics[0], calibration.extrinsics[1], calibration.extrinsics[2], calibration.extrinsics[3], calibration.extrinsics[4], calibration.extrinsics[5], calibration.extrinsics[6], calibration.extrinsics[7], calibration.extrinsics[8], calibration.extrinsics[9], calibration.extrinsics[10], calibration.extrinsics[11], calibration.extrinsics[12], calibration.extrinsics[13], calibration.extrinsics[14], calibration.extrinsics[15]));

        hl2ss.svc.open_stream(host, port, 300, configuration, true, out source_rm_vlc);

        tex_vlc  = new Texture2D(hl2ss.parameters_rm_vlc.WIDTH, hl2ss.parameters_rm_vlc.HEIGHT, TextureFormat.R8, false);
        texr_vlc = new RenderTexture(hl2ss.parameters_rm_vlc.WIDTH, hl2ss.parameters_rm_vlc.HEIGHT, 0, RenderTextureFormat.BGRA32);

        quad_rm_vlc.GetComponent<Renderer>().material.mainTexture = texr_vlc;

        mat_vlc = new Material(shader_vlc);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_rm_vlc.get_by_index(-1);

        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_rm_vlc region);

        var metadata = Marshal.PtrToStructure<hl2ss.rm_vlc_metadata>(region.metadata);
        var pose     = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));
        Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose.m_00, pose.m_01, pose.m_02, pose.m_03, pose.m_10, pose.m_11, pose.m_12, pose.m_13, pose.m_20, pose.m_21, pose.m_22, pose.m_23, pose.m_30, pose.m_31, pose.m_32, pose.m_33));

        Debug.Log(string.Format("sensor_ticks {0}", metadata.sensor_ticks));
        Debug.Log(string.Format("exposure {0}",     metadata.exposure));
        Debug.Log(string.Format("gain {0}",         metadata.gain));

        tex_vlc.LoadRawTextureData(region.image, (int)hl2ss.parameters_rm_vlc.PIXELS * sizeof(byte));
        tex_vlc.Apply();

        Graphics.Blit(tex_vlc, texr_vlc, mat_vlc);
    }
}
