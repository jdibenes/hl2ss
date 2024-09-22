
using System.Runtime.InteropServices;
using UnityEngine;

public class test_rm_depth_ahat : MonoBehaviour
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

    private hl2ss.svc.source source_rm_depth_ahat;

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;
        ushort port = hl2ss.stream_port.RM_DEPTH_AHAT;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_rm_depth_ahat configuration);

        using (var calibration_handle = hl2ss.svc.download_calibration(host, port, configuration))
        {
            var calibration = Marshal.PtrToStructure<hl2ss.calibration_rm_depth_ahat>(calibration_handle.data);
        }

        source_rm_depth_ahat = hl2ss.svc.open_stream(host, port, 450, configuration);

        tex_z  = new Texture2D(hl2ss.parameters_rm_depth_ahat.WIDTH, hl2ss.parameters_rm_depth_ahat.HEIGHT, TextureFormat.R16, false);
        tex_ab = new Texture2D(hl2ss.parameters_rm_depth_ahat.WIDTH, hl2ss.parameters_rm_depth_ahat.HEIGHT, TextureFormat.R16, false);

        texr_z  = new RenderTexture(hl2ss.parameters_rm_depth_ahat.WIDTH, hl2ss.parameters_rm_depth_ahat.HEIGHT, 0, RenderTextureFormat.BGRA32);
        texr_ab = new RenderTexture(hl2ss.parameters_rm_depth_ahat.WIDTH, hl2ss.parameters_rm_depth_ahat.HEIGHT, 0, RenderTextureFormat.BGRA32);

        quad_z.GetComponent<Renderer>().material.mainTexture  = texr_z;
        quad_ab.GetComponent<Renderer>().material.mainTexture = texr_ab;

        mat_z  = new Material(shader_z);
        mat_ab = new Material(shader_ab);

        mat_z.SetTexture("_ColorMapTex", colormap_z);
        mat_z.SetFloat("_Lf", 0.0f / 65535.0f);
        mat_z.SetFloat("_Rf", 1055.0f / 65535.0f);
    }

    // Update is called once per frame
    void Update()
    {
        using (var packet = source_rm_depth_ahat.get_by_index(-1))
        {
            if (packet.status != 0) { return; }

            packet.unpack(out hl2ss.map_rm_depth_ahat region);

            var metadata = Marshal.PtrToStructure<hl2ss.rm_depth_ahat_metadata>(region.metadata);
            var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

            tex_z.LoadRawTextureData(region.depth, (int)hl2ss.parameters_rm_depth_ahat.PIXELS * sizeof(ushort));
            tex_z.Apply();

            tex_ab.LoadRawTextureData(region.ab, (int)hl2ss.parameters_rm_depth_ahat.PIXELS * sizeof(ushort));
            tex_ab.Apply();

            Graphics.Blit(tex_z, texr_z, mat_z);
            Graphics.Blit(tex_ab, texr_ab, mat_ab);
        }
    }

    void OnApplicationQuit()
    {
        if (source_rm_depth_ahat == null) { return; }

        source_rm_depth_ahat.Dispose();
    }
}
