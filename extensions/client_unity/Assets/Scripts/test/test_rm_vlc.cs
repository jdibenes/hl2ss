
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

    private hl2ss.svc.source source_rm_vlc;    

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_rm_vlc configuration);

        using (var calibration_handle = hl2ss.svc.download_calibration(host, port, configuration))
        {
            var calibration = Marshal.PtrToStructure<hl2ss.calibration_rm_vlc>(calibration_handle.data);
        }

        source_rm_vlc = hl2ss.svc.open_stream(host, port, 300, configuration);

        tex_vlc = new Texture2D(hl2ss.parameters_rm_vlc.WIDTH, hl2ss.parameters_rm_vlc.HEIGHT, TextureFormat.R8, false);
        texr_vlc = new RenderTexture(hl2ss.parameters_rm_vlc.WIDTH, hl2ss.parameters_rm_vlc.HEIGHT, 0, RenderTextureFormat.BGRA32);

        quad_rm_vlc.GetComponent<Renderer>().material.mainTexture = texr_vlc;

        mat_vlc = new Material(shader_vlc);
    }

    // Update is called once per frame
    void Update()
    {
        using (var packet = source_rm_vlc.get_by_index(-1))
        {
            if (packet.status != 0) { return; }

            packet.unpack(out hl2ss.map_rm_vlc region);

            var metadata = Marshal.PtrToStructure<hl2ss.rm_vlc_metadata>(region.metadata);
            var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

            tex_vlc.LoadRawTextureData(region.image, (int)hl2ss.parameters_rm_vlc.PIXELS * sizeof(byte));
            tex_vlc.Apply();

            Graphics.Blit(tex_vlc, texr_vlc, mat_vlc);
        }
    }

    void OnApplicationQuit()
    {
        if (source_rm_vlc == null) { return; }

        source_rm_vlc.Dispose();
    }
}
