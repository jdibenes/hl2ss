
using System.Runtime.InteropServices;
using UnityEngine;

public class test_rm_depth_longthrow : MonoBehaviour
{
    public GameObject quad_z;
    public Shader shader_z;
    public Texture2D colormap_z;
    public GameObject quad_ab;
    public Shader shader_ab;

    private Texture2D tex_z;
    private Texture2D tex_ab;
    private RenderTexture texr_z;
    private RenderTexture texr_ab;
    private Material mat_z;
    private Material mat_ab;

    hl2ss.shared.source source_rm_depth_longthrow;

    // Start is called before the first frame update
    void Start()
    {
        var host = run_once.host_address;
        var port = hl2ss.stream_port.RM_DEPTH_LONGTHROW;

        var configuration = new hl2ss.ulm.configuration_rm_depth_longthrow();

        using var calibration_handle = hl2ss.svc.download_calibration(host, port, configuration);
        var calibration = Marshal.PtrToStructure<hl2ss.calibration_rm_depth_longthrow>(calibration_handle.data);

        Debug.Log(string.Format("intrinsics fx {0} fy {1} cx {2} cy {3}", calibration.intrinsics[0], calibration.intrinsics[1], calibration.intrinsics[2], calibration.intrinsics[3]));
        Debug.Log(string.Format("extrinsics [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", calibration.extrinsics[0], calibration.extrinsics[1], calibration.extrinsics[2], calibration.extrinsics[3], calibration.extrinsics[4], calibration.extrinsics[5], calibration.extrinsics[6], calibration.extrinsics[7], calibration.extrinsics[8], calibration.extrinsics[9], calibration.extrinsics[10], calibration.extrinsics[11], calibration.extrinsics[12], calibration.extrinsics[13], calibration.extrinsics[14], calibration.extrinsics[15]));
        Debug.Log(string.Format("scale {0}", calibration.scale));

        hl2ss.svc.open_stream(host, port, 50, configuration, true, out source_rm_depth_longthrow);

        tex_z  = new Texture2D(hl2ss.parameters_rm_depth_longthrow.WIDTH, hl2ss.parameters_rm_depth_longthrow.HEIGHT, TextureFormat.R16, false);
        tex_ab = new Texture2D(hl2ss.parameters_rm_depth_longthrow.WIDTH, hl2ss.parameters_rm_depth_longthrow.HEIGHT, TextureFormat.R16, false);

        texr_z  = new RenderTexture(hl2ss.parameters_rm_depth_longthrow.WIDTH, hl2ss.parameters_rm_depth_longthrow.HEIGHT, 0, RenderTextureFormat.BGRA32);
        texr_ab = new RenderTexture(hl2ss.parameters_rm_depth_longthrow.WIDTH, hl2ss.parameters_rm_depth_longthrow.HEIGHT, 0, RenderTextureFormat.BGRA32);

        quad_z.GetComponent<Renderer>().material.mainTexture  = texr_z;
        quad_ab.GetComponent<Renderer>().material.mainTexture = texr_ab;

        mat_z  = new Material(shader_z);
        mat_ab = new Material(shader_ab);

        mat_z.SetTexture("_ColorMapTex", colormap_z);
        mat_z.SetFloat("_Lf", 0.0f / 65535.0f);
        mat_z.SetFloat("_Rf", 7500.0f / 65535.0f);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_rm_depth_longthrow.get_by_index(-1);
        
        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_rm_depth_longthrow region);

        var metadata = Marshal.PtrToStructure<hl2ss.rm_depth_longthrow_metadata>(region.metadata);
        var pose     = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));
        Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose.m_00, pose.m_01, pose.m_02, pose.m_03, pose.m_10, pose.m_11, pose.m_12, pose.m_13, pose.m_20, pose.m_21, pose.m_22, pose.m_23, pose.m_30, pose.m_31, pose.m_32, pose.m_33));

        Debug.Log(string.Format("sensor_ticks {0}", metadata.sensor_ticks));

        tex_z.LoadRawTextureData(region.depth, (int)hl2ss.parameters_rm_depth_longthrow.PIXELS * sizeof(ushort));
        tex_z.Apply();

        tex_ab.LoadRawTextureData(region.ab, (int)hl2ss.parameters_rm_depth_longthrow.PIXELS * sizeof(ushort));
        tex_ab.Apply();

        Graphics.Blit(tex_z, texr_z, mat_z);
        Graphics.Blit(tex_ab, texr_ab, mat_ab);
    }
}
