
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

        var configuration = new hl2ss.ulm.configuration_pv();

        configuration.width = 640;
        configuration.height = 360;
        configuration.framerate = 30;

        var decoded_format = hl2ss.pv_decoded_format.RGB;
        texture_format = TextureFormat.RGB24;
        bpp = 3;

        var configuration_subsystem = new hl2ss.ulm.configuration_pv_subsystem();

        hl2ss.svc.start_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO, configuration_subsystem);

        using var calibration_handle = hl2ss.svc.download_calibration(host, hl2ss.stream_port.PERSONAL_VIDEO, configuration);
        var calibration = Marshal.PtrToStructure<hl2ss.calibration_pv>(calibration_handle.data);

        Debug.Log(string.Format("fx {0} fy {1} cx {2} cy {3}", calibration.focal_length[0], calibration.focal_length[1], calibration.principal_point[0], calibration.principal_point[1]));

        hl2ss.svc.open_stream(host, hl2ss.stream_port.PERSONAL_VIDEO, 300, configuration, decoded_format, out source_pv);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_pv.get_by_index(-1);

        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_pv region);

        var metadata = Marshal.PtrToStructure<hl2ss.pv_metadata>(region.metadata);
        var pose     = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));
        Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose.m_00, pose.m_01, pose.m_02, pose.m_03, pose.m_10, pose.m_11, pose.m_12, pose.m_13, pose.m_20, pose.m_21, pose.m_22, pose.m_23, pose.m_30, pose.m_31, pose.m_32, pose.m_33));

        Debug.Log(string.Format("fx {0} fy {1} cx {2} cy {3}", metadata.f.x, metadata.f.y, metadata.c.x, metadata.c.y));
        Debug.Log(string.Format("exposure_time {0} exposure_compensation [{1}, {2}]", metadata.exposure_time, metadata.exposure_compensation.val_0, metadata.exposure_compensation.val_1));
        Debug.Log(string.Format("focus_state {0} lens_position {1}", metadata.focus_state, metadata.lens_position));
        Debug.Log(string.Format("iso_speed {0} iso_gains [{1}, {2}]", metadata.iso_speed, metadata.iso_gains.x, metadata.iso_gains.y));
        Debug.Log(string.Format("white_balance {0} white_balance_gains [{1}, {2}, {3}]", metadata.white_balance, metadata.white_balance_gains.x, metadata.white_balance_gains.y, metadata.white_balance_gains.z));
        Debug.Log(string.Format("resolution {0}x{1}", metadata.width, metadata.height));

        if (!tex_pv)
        {
            var width  = metadata.width;
            var height = metadata.height;

            pv_frame_size = width * height * bpp;
            tex_pv = new Texture2D(width, height, texture_format, false);
            quad_pv.GetComponent<Renderer>().material.mainTexture = tex_pv;
        }

        tex_pv.LoadRawTextureData(region.image, pv_frame_size);
        tex_pv.Apply();
    }

    void OnApplicationQuit()
    {
        if (source_pv == null) { return; }

        source_pv.Dispose();
        hl2ss.svc.stop_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);
    }
}
