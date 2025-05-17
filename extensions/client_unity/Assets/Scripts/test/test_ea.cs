
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

public class test_ea : MonoBehaviour
{
    public GameObject audio_source_object;
    private hl2ss.shared.source source_ea;
    private AudioSource audio_source;
    private long index;
    private List<float> buffer;

    // Start is called before the first frame update
    void Start()
    {
        var host = run_once.host_address;

        var configuration = new hl2ss.ulm.configuration_extended_audio();

        using var device_list_handle = hl2ss.svc.download_device_list(host, hl2ss.stream_port.EXTENDED_AUDIO, configuration);

        var string_bytes = new byte[device_list_handle.size];
        Marshal.Copy(device_list_handle.data, string_bytes, 0, (int)device_list_handle.size);
        Debug.Log(Encoding.Unicode.GetString(string_bytes));

        hl2ss.svc.open_stream(host, hl2ss.stream_port.EXTENDED_AUDIO, 1000, configuration, true, out source_ea);
        index = -1;

        buffer = new List<float>();

        audio_source = audio_source_object.GetComponent<AudioSource>();
        audio_source.clip = AudioClip.Create("audio_mc", 4 * hl2ss.parameters_microphone.GROUP_SIZE_AAC, hl2ss.parameters_microphone.CHANNELS, (int)hl2ss.parameters_microphone.SAMPLE_RATE, true, OnAudioRead);
        audio_source.Play();
    }

    void OnAudioRead(float[] data)
    {
        int count = 0;
        
        while ((count < data.Length) && (count < buffer.Count))
        {
            data[count] = buffer[count];
            count++;
        }
        buffer.RemoveRange(0, count);
        while (count < data.Length)
        {
            data[count] = 0.0f;
            count++;
        }
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_ea.get_by_index(index);

        switch (packet.status)
        {
        case hl2ss.mt.status.DISCARDED: index = -1;                     return;
        case hl2ss.mt.status.WAIT:                                      return;
        case hl2ss.mt.status.OK:        index = packet.frame_stamp + 1; break;
        }

        packet.unpack<float>(out hl2ss.map_microphone region);

        var samples = new float[region.count];
        Marshal.Copy(region.samples, samples, 0, samples.Length);

        buffer.AddRange(hl2ss.microphone_planar_to_packed<float>(samples, hl2ss.parameters_microphone.CHANNELS));
    }
}
