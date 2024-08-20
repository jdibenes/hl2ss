
using System.Runtime.InteropServices;
using UnityEngine;
using static hl2ss.ulm;

public class test_rm_vlc : MonoBehaviour
{
    public GameObject quad_rm_vlc;
    public ushort port = hl2ss.stream_port.RM_VLC_LEFTFRONT;

    private hl2ss.svc.source source_rm_vlc;
    private Texture2D texture_rm_vlc;

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_rm_vlc configuration);

        var calibration_handle = hl2ss.svc.download_calibration(host, port, configuration);
        var calibration = Marshal.PtrToStructure<hl2ss.calibration_rm_vlc>(calibration_handle.data);
        calibration_handle.destroy();

        source_rm_vlc = hl2ss.svc.open_stream(host, port, 300, configuration);

        texture_rm_vlc = new Texture2D(hl2ss.parameters_rm_vlc.WIDTH, hl2ss.parameters_rm_vlc.HEIGHT, TextureFormat.R8, false);
        quad_rm_vlc.GetComponent<Renderer>().material.mainTexture = texture_rm_vlc;
    }

    // Update is called once per frame
    void Update()
    {
        var packet = source_rm_vlc.get_by_index(-1);
        if (packet.status != 0) { return; }

        packet.unpack(out hl2ss.map_rm_vlc region);

        var metadata = Marshal.PtrToStructure<hl2ss.rm_vlc_metadata>(region.metadata);
        var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        texture_rm_vlc.LoadRawTextureData(region.image, (int)hl2ss.parameters_rm_vlc.PIXELS * sizeof(byte));
        texture_rm_vlc.Apply();

        packet.destroy();
    }

    void OnApplicationQuit()
    {
        if (source_rm_vlc == null) { return; }

        source_rm_vlc.destroy();
    }
}
