
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
    public ushort width = 640;
    public ushort height = 360;

    private string host;
    private hl2ss.svc.source source_ez;
    private int ez_frame_size;
    private Texture2D tex_z;
    private RenderTexture texr_z;
    private Material mat_z;

    // Start is called before the first frame update
    void Start()
    {
        host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_extended_depth configuration);

        configuration.media_index = media_index;
        ez_frame_size = width * height * sizeof(ushort);

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_pv_subsystem configuration_subsystem);

        configuration_subsystem.global_opacity = group_index;
        configuration_subsystem.output_width   = source_index;
        configuration_subsystem.output_height  = profile_index;

        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.EXTENDED_DEPTH, configuration_subsystem);

        source_ez = hl2ss.svc.open_stream(host, hl2ss.stream_port.EXTENDED_DEPTH, 300, configuration);

        tex_z = new Texture2D(width, height, TextureFormat.R16, false);
        texr_z = new RenderTexture(width, height, 0, RenderTextureFormat.BGRA32);
        quad_ez.GetComponent<Renderer>().material.mainTexture = texr_z;
        mat_z = new Material(shader_z);
        mat_z.SetTexture("_ColorMapTex", colormap_z);
        mat_z.SetFloat("_Lf", 0.0f / 65535.0f);
        mat_z.SetFloat("_Rf", 4096.0f / 65535.0f);
    }

    // Update is called once per frame
    void Update()
    {
        using (var packet = source_ez.get_by_index(-1))
        {
            if (packet.status != 0) { return; }

            packet.unpack(out hl2ss.map_extended_depth region);

            var metadata = Marshal.PtrToStructure<hl2ss.extended_depth_metadata>(region.metadata);

            tex_z.LoadRawTextureData(region.depth, ez_frame_size);
            tex_z.Apply();

            Graphics.Blit(tex_z, texr_z, mat_z);
        }
    }

    void OnApplicationQuit()
    {
        if (source_ez == null) { return; }

        source_ez.Dispose();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.EXTENDED_DEPTH);
    }
}
