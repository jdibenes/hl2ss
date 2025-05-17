
using System.Runtime.InteropServices;
using UnityEngine;

public class test_pv : MonoBehaviour
{
    public GameObject quad_pv;
    private string host;
    private hl2ss.shared.source source_pv;
    private int pv_frame_size;
    private Texture2D tex_pv;
    private TextureFormat texture_format;
    private int bpp;

    // Start is called before the first frame update
    void Start()
    {
        host = run_once.host_address;

        hl2ss.ulm.configuration_pv configuration = new hl2ss.ulm.configuration_pv();

        configuration.width = 640;
        configuration.height = 360;
        configuration.framerate = 30;
        byte decoded_format = hl2ss.pv_decoded_format.RGB;

        switch (decoded_format)
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

        hl2ss.ulm.configuration_pv_subsystem configuration_subsystem = new hl2ss.ulm.configuration_pv_subsystem();

        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO, configuration_subsystem);

        using (var calibration_handle = hl2ss.svc.download_calibration(host, hl2ss.stream_port.PERSONAL_VIDEO, configuration))
        {
            var calibration = Marshal.PtrToStructure<hl2ss.calibration_pv>(calibration_handle.data);
        }

        hl2ss.svc.open_stream(host, hl2ss.stream_port.PERSONAL_VIDEO, 300, configuration, decoded_format, out source_pv);
    }

    // Update is called once per frame
    void Update()
    {
        using (var packet = source_pv.get_by_index(-1))
        {
            if (packet.status != 0) { return; }
            packet.unpack(out hl2ss.map_pv region);

            hl2ss.pv_metadata metadata = Marshal.PtrToStructure<hl2ss.pv_metadata>(region.metadata);
            hl2ss.matrix_4x4 pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

            if (!tex_pv)
            {
                ushort width = metadata.width;
                ushort height = metadata.height;
                Debug.Log(string.Format("pv dimensions: {0} x {1}", width, height));
                pv_frame_size = width * height * bpp;
                tex_pv = new Texture2D(width, height, texture_format, false);
                quad_pv.GetComponent<Renderer>().material.mainTexture = tex_pv;
            }

            tex_pv.LoadRawTextureData(region.image, pv_frame_size);
            tex_pv.Apply();
        }
    }

    void OnApplicationQuit()
    {
        if (source_pv == null) { return; }

        source_pv.Dispose();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);
    }
}
