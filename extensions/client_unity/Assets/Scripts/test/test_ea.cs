
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

public class test_ea : MonoBehaviour
{
    public GameObject audio_source_object;
    private hl2ss.svc.source source_ea;
    private AudioSource audio_source;
    private long index;
    private List<float> buffer;

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_extended_audio configuration);

        using (var device_list_handle = hl2ss.svc.download_device_list(host, hl2ss.stream_port.EXTENDED_AUDIO))
        {
            var string_bytes = new byte[device_list_handle.size];
            Marshal.Copy(device_list_handle.data, string_bytes, 0, (int)device_list_handle.size);
            Debug.Log(Encoding.Unicode.GetString(string_bytes));
        }

        source_ea = hl2ss.svc.open_stream(host, hl2ss.stream_port.EXTENDED_AUDIO, 1000, configuration);
        index = 0;

        buffer = new List<float>();
        audio_source = audio_source_object.GetComponent<AudioSource>();
        audio_source.clip = AudioClip.Create("audio_mc", 4 * hl2ss.parameters_extended_audio.GROUP_SIZE_AAC, hl2ss.parameters_extended_audio.CHANNELS, (int)hl2ss.parameters_extended_audio.SAMPLE_RATE, true, OnAudioRead);
        audio_source.Play();
    }

    void OnAudioRead(float[] data)
    {
        int count = 0;
        
        while (count < data.Length)
        {
            if (count >= buffer.Count) { break; }
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
        using (var packet = source_ea.get_by_index(index))
        {
            if (packet.status < 0)
            {
                index++;
                return;
            }
            if (packet.status > 0)
            {
                return;
            }

            index++;

            packet.unpack(out hl2ss.map_extended_audio_aac region);

            float[] b = new float[packet.sz_payload / sizeof(float)];
            Marshal.Copy(region.samples, b, 0, b.Length);
            for (int i = 0; i < (b.Length / 2); ++i)
            {
                buffer.Add(b[i]);
                buffer.Add(b[(b.Length / 2) + i]);
            }
        }
    }
}
