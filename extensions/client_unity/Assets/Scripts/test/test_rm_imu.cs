
using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class test_rm_imu : MonoBehaviour
{
    public ushort port = hl2ss.stream_port.RM_IMU_ACCELEROMETER;

    private hl2ss.shared.source source_rm_imu;

    // Start is called before the first frame update
    void Start()
    {
        var host = run_once.host_address;

        var configuration = new hl2ss.ulm.configuration_rm_imu();

        if (port != hl2ss.stream_port.RM_IMU_MAGNETOMETER)
        {
            using var calibration_handle = hl2ss.svc.download_calibration(host, port, configuration);
            var calibration = Marshal.PtrToStructure<hl2ss.calibration_rm_imu>(calibration_handle.data);
            Debug.Log(string.Format("extrinsics [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", calibration.extrinsics[0], calibration.extrinsics[1], calibration.extrinsics[2], calibration.extrinsics[3], calibration.extrinsics[4], calibration.extrinsics[5], calibration.extrinsics[6], calibration.extrinsics[7], calibration.extrinsics[8], calibration.extrinsics[9], calibration.extrinsics[10], calibration.extrinsics[11], calibration.extrinsics[12], calibration.extrinsics[13], calibration.extrinsics[14], calibration.extrinsics[15]));
        }

        hl2ss.svc.open_stream(host, port, 1000, configuration, true, out source_rm_imu);
    }

    // Update is called once per frame
    void Update()
    {
        using var packet = source_rm_imu.get_by_index(-1);

        if (packet.status != hl2ss.mt.status.OK) { return; }

        packet.unpack(out hl2ss.map_rm_imu region);

        var pose = Marshal.PtrToStructure<hl2ss.matrix_4x4>(packet.pose);

        var samples = new hl2ss.rm_imu_sample[region.count];
        for (int i = 0; i < region.count; ++i) { samples[i] = Marshal.PtrToStructure<hl2ss.rm_imu_sample>(IntPtr.Add(region.samples, i * Marshal.SizeOf<hl2ss.rm_imu_sample>())); }

        Debug.Log(string.Format("timestamp {0}", packet.timestamp));
        Debug.Log(string.Format("pose [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose.m_00, pose.m_01, pose.m_02, pose.m_03, pose.m_10, pose.m_11, pose.m_12, pose.m_13, pose.m_20, pose.m_21, pose.m_22, pose.m_23, pose.m_30, pose.m_31, pose.m_32, pose.m_33));

        Debug.Log(string.Format("got {0} samples", region.count));
        Debug.Log(string.Format("first sample timestamp {0} sensor_timestamp {1} x {2} y {3} z {4} temperature {5}]",  samples[0].timestamp, samples[0].sensor_timestamp, samples[0].x, samples[0].y, samples[0].z, samples[0].temperature));
        Debug.Log(string.Format("second sample timestamp {0} sensor_timestamp {1} x {2} y {3} z {4} temperature {5}]", samples[1].timestamp, samples[1].sensor_timestamp, samples[1].x, samples[1].y, samples[1].z, samples[1].temperature));
    }
}
