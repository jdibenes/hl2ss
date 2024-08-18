
using System.Runtime.InteropServices;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

public class viewer : MonoBehaviour
{
    public GameObject quad_pv;
    private string host;
    private hl2ss.svc.source source_pv;
    private int pv_frame_size;
    private Texture2D tex_pv;

    // Start is called before the first frame update
    void Start()
    {
        host = run_once.host_address;

        hl2ss.ulm.configuration_pv configuration = hl2ss.svc.create_configuration_pv();

        configuration.width = 1280;
        configuration.height = 720;
        configuration.framerate = 30;
        configuration.decoded_format = hl2ss.pv_decoded_format.RGB;

        TextureFormat texture_format;
        int bpp;

        switch (configuration.decoded_format)
        {
        case hl2ss.pv_decoded_format.RGB:
            texture_format = TextureFormat.RGB24;
            bpp = 3;
            break;
        case hl2ss.pv_decoded_format.BGR:
            texture_format = TextureFormat.RGB24; // no BGR24 ?
            bpp = 3;
            break;
        case hl2ss.pv_decoded_format.RGBA:
            texture_format = TextureFormat.RGBA32;
            bpp = 4;
            break;
        case hl2ss.pv_decoded_format.BGRA:
            texture_format = TextureFormat.BGRA32;
            bpp = 4;
            break;
        case hl2ss.pv_decoded_format.GRAY:
            texture_format = TextureFormat.R8;
            bpp = 1;
            break;
        default:
            throw new System.Exception("Unsupported decoded format");
        }

        pv_frame_size = configuration.width * configuration.height * bpp;

        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);

        var calibration_handle = hl2ss.svc.download_calibration(host, hl2ss.stream_port.PERSONAL_VIDEO, configuration);
        var calibration = Marshal.PtrToStructure<hl2ss.calibration_pv>(calibration_handle.address);
        calibration_handle.destroy();

        source_pv = hl2ss.svc.open_stream(host, hl2ss.stream_port.PERSONAL_VIDEO, 300, configuration);

        tex_pv = new Texture2D(configuration.width, configuration.height, texture_format, false);

        quad_pv.GetComponent<Renderer>().material.mainTexture = tex_pv;
    }

    // Update is called once per frame
    void Update()
    {
        hl2ss.svc.packet packet = source_pv.get_by_index(-1);
        if (packet.status != 0) { return; }
        packet.unpack(out hl2ss.map_pv region);

        hl2ss.pv_metadata metadata = Marshal.PtrToStructure<hl2ss.pv_metadata>(region.metadata);
        hl2ss.matrix_4x4 pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        Debug.Log(packet.frame_stamp);
        Debug.Log(packet.timestamp);
        Debug.Log(metadata.f.x);
        Debug.Log(metadata.lens_position);
        Debug.Log(metadata.focus_state);
        Debug.Log(metadata.white_balance_gains.x);
        Debug.Log(metadata.white_balance_gains.y);
        Debug.Log(metadata.white_balance_gains.z);
        Debug.Log(pose.m[15]);

        tex_pv.LoadRawTextureData(region.image, pv_frame_size);
        tex_pv.Apply();

        packet.destroy();
    }

    void OnApplicationQuit()
    {
        source_pv.destroy();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);
    }
}
