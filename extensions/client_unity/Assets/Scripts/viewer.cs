using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class viewer : MonoBehaviour
{
    public string host;
    public GameObject quad_pv;

    private Texture2D tex_pv;

    // Start is called before the first frame update
    void Start()
    {
        ushort pv_width = 1280;
        ushort pv_height = 720;
        byte pv_framerate = 30;

        hl2ss.ulm.initialize();
        hl2ss.ulm.start_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);
        hl2ss.ulm.open_pv(host, hl2ss.stream_port.PERSONAL_VIDEO, pv_width, pv_height, pv_framerate, hl2ss.chunk_size.PERSONAL_VIDEO, hl2ss.stream_mode.MODE_1, 1, hl2ss.video_profile.H265_MAIN, hl2ss.h26x_level.DEFAULT, 0, 0, null, hl2ss.pv_decoded_format.RGB);

        tex_pv = new Texture2D(pv_width, pv_height, TextureFormat.RGB24, false);

        quad_pv.GetComponent<Renderer>().material.mainTexture = tex_pv;
    }

    // Update is called once per frame
    void Update()
    {
        long frame_stamp = -1;
        int status = -1;
        IntPtr frame = IntPtr.Zero;
        int valid = hl2ss.ulm.get_by_index(hl2ss.stream_port.PERSONAL_VIDEO, ref frame_stamp, ref status, ref frame);
        if (valid < 0) { return; }
        if (status != 0) { return; }

        ulong timestamp = 0;
        uint payload_size = 0;
        IntPtr payload_data = IntPtr.Zero;
        IntPtr pose_data = IntPtr.Zero;
        IntPtr pv_image = IntPtr.Zero;
        IntPtr pv_intrinsics = IntPtr.Zero;
        hl2ss.ulm.unpack_frame(frame, ref timestamp, ref payload_size, ref payload_data, ref pose_data);
        hl2ss.ulm.unpack_pv(payload_data, payload_size, ref pv_image, ref pv_intrinsics);

        tex_pv.LoadRawTextureData(payload_data, (int)payload_size);
        tex_pv.Apply();

        hl2ss.ulm.release_frame(frame);
    }

    void OnApplicationQuit()
    {
        hl2ss.ulm.close(hl2ss.stream_port.PERSONAL_VIDEO);
        hl2ss.ulm.stop_subsystem_pv(host, hl2ss.stream_port.PERSONAL_VIDEO);        
    }
}
