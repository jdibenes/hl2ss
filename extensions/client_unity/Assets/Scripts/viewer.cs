
using UnityEngine;

public class viewer : MonoBehaviour
{
    public string host;
    public GameObject quad_pv;
    public hl2ss.svc.source source_pv; 

    private Texture2D tex_pv;

    // Start is called before the first frame update
    void Start()
    {
        hl2ss.ulm.configuration_pv configuration = hl2ss.svc.create_configuration_pv();

        configuration.width = 1280;
        configuration.height = 720;
        configuration.framerate = 30;
        configuration.decoded_format = hl2ss.pv_decoded_format.RGB;

        hl2ss.svc.initialize();
        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);

        source_pv = hl2ss.svc.open_stream(host, hl2ss.stream_port.PERSONAL_VIDEO, 300, configuration);

        tex_pv = new Texture2D(configuration.width, configuration.height, TextureFormat.RGB24, false);

        quad_pv.GetComponent<Renderer>().material.mainTexture = tex_pv;
    }

    // Update is called once per frame
    void Update()
    {
        hl2ss.svc.packet packet = source_pv.get_by_index(-1);
        if (packet.status != 0) { return; }

        tex_pv.LoadRawTextureData(packet.payload, (int)packet.sz_payload);
        tex_pv.Apply();
    }

    void OnApplicationQuit()
    {
        source_pv.destroy();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);
    }
}
