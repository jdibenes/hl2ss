
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
    private hl2ss.svc.source source_pv;
    private int pv_frame_size;
    private Texture2D tex_pv;

    // Start is called before the first frame update
    void Start()
    {
        host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_pv configuration);

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

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_pv_subsystem configuration_subsystem);

        configuration_subsystem.global_opacity = group_index;
        configuration_subsystem.output_width   = source_index;
        configuration_subsystem.output_height  = profile_index;

        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.EXTENDED_VIDEO, configuration_subsystem);

        using (var device_list_handle = hl2ss.svc.download_device_list(host, hl2ss.stream_port.EXTENDED_VIDEO))
        {
            var string_bytes = new byte[device_list_handle.size];
            Marshal.Copy(device_list_handle.data, string_bytes, 0, (int)device_list_handle.size);
            Debug.Log(Encoding.Unicode.GetString(string_bytes));
        }

        source_pv = hl2ss.svc.open_stream(host, hl2ss.stream_port.EXTENDED_VIDEO, 300, configuration);

        tex_pv = new Texture2D(configuration.width, configuration.height, texture_format, false);

        quad_pv.GetComponent<Renderer>().material.mainTexture = tex_pv;
    }

    // Update is called once per frame
    void Update()
    {
        using (hl2ss.svc.packet packet = source_pv.get_by_index(-1))
        {
            if (packet.status != 0) { return; }
            packet.unpack(out hl2ss.map_pv region);

            tex_pv.LoadRawTextureData(region.image, pv_frame_size);
            tex_pv.Apply();
        }
    }

    void OnApplicationQuit()
    {
        if (source_pv == null) { return; }

        source_pv.Dispose();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.EXTENDED_VIDEO);
    }
}
