
using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class test_rm_imu : MonoBehaviour
{
    public ushort port = hl2ss.stream_port.RM_IMU_ACCELEROMETER;

    private hl2ss.svc.source source_rm_imu;

    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.create_configuration(out hl2ss.ulm.configuration_rm_imu configuration);

        if (port != hl2ss.stream_port.RM_IMU_MAGNETOMETER)
        {
            using (var calibration_handle = hl2ss.svc.download_calibration(host, port, configuration))
            {
                var calibration = Marshal.PtrToStructure<hl2ss.calibration_rm_imu>(calibration_handle.data);
            }
        }

        source_rm_imu = hl2ss.svc.open_stream(host, port, 1000, configuration);
    }

    // Update is called once per frame
    void Update()
    {
        using (var packet = source_rm_imu.get_by_index(-1))
        {
            if (packet.status != 0) { return; }

            packet.unpack(out hl2ss.map_rm_imu region);

            var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

            Debug.Log(string.Format("got {0} samples", packet.sz_payload / Marshal.SizeOf<hl2ss.rm_imu_sample>()));

            var sample = Marshal.PtrToStructure<hl2ss.rm_imu_sample>(IntPtr.Add(region.samples, 0));

            Debug.Log(string.Format("first sample [{0}, {1}, {2}]", sample.x, sample.y, sample.z));
        }
    }

    void OnApplicationQuit()
    {
        if (source_rm_imu == null) { return; }

        source_rm_imu.Dispose();
    }
}
