
using System.Runtime.InteropServices;
using UnityEngine;

public class test_extended_depth : MonoBehaviour
{
    public GameObject quad_ez;
    public Shader shader_z;
    public Texture2D colormap_z;
    public float group_index = 0.0f;
    public float source_index = 0.0f;
    public float profile_index = 0.0f;
    public ulong media_index = 15;

    private string host;
    private hl2ss.shared.source source_ez;
    private int ez_frame_size;
    private Texture2D tex_z;
    private RenderTexture texr_z;
    private Material mat_z;

    // Start is called before the first frame update
    void Start()
    {
        host = run_once.host_address;

        var configuration = new hl2ss.ulm.configuration_extended_depth();

        configuration.media_index = media_index;

        var configuration_subsystem = new hl2ss.ulm.configuration_pv_subsystem();

        configuration_subsystem.global_opacity = group_index;
        configuration_subsystem.output_width   = source_index;
        configuration_subsystem.output_height  = profile_index;

        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.EXTENDED_DEPTH, configuration_subsystem);

        hl2ss.svc.open_stream(host, hl2ss.stream_port.EXTENDED_DEPTH, 300, configuration, true, out source_ez);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_ez.get_by_index(-1);

        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_extended_depth region);

        var metadata = Marshal.PtrToStructure<hl2ss.extended_depth_metadata>(region.metadata);
        var pose     = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));
        Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose.m_00, pose.m_01, pose.m_02, pose.m_03, pose.m_10, pose.m_11, pose.m_12, pose.m_13, pose.m_20, pose.m_21, pose.m_22, pose.m_23, pose.m_30, pose.m_31, pose.m_32, pose.m_33));

        Debug.Log(string.Format("resolution {0}x{1}", metadata.width, metadata.height));

        if (!tex_z)
        {
            var width  = metadata.width;
            var height = metadata.height;

            ez_frame_size = width * height * sizeof(ushort);

            tex_z  = new Texture2D(width, height, TextureFormat.R16, false);
            texr_z = new RenderTexture(width, height, 0, RenderTextureFormat.BGRA32);

            quad_ez.GetComponent<Renderer>().material.mainTexture = texr_z;

            mat_z = new Material(shader_z);

            mat_z.SetTexture("_ColorMapTex", colormap_z);
            mat_z.SetFloat("_Lf", 0.0f / 65535.0f);
            mat_z.SetFloat("_Rf", 8192.0f / 65535.0f);
        }

        tex_z.LoadRawTextureData(region.depth, ez_frame_size);
        tex_z.Apply();

        Graphics.Blit(tex_z, texr_z, mat_z);
    }

    void OnApplicationQuit()
    {
        if (source_ez == null) { return; }

        source_ez.Dispose();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.EXTENDED_DEPTH);
    }
}
