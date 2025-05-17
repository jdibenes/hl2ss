
using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class test_ipc_rc : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        hl2ss.svc.open_ipc(run_once.host_address, hl2ss.ipc_port.REMOTE_CONFIGURATION, out hl2ss.shared.ipc_rc ipc);

        hl2ss.version version = ipc.ee_get_application_version();
        Debug.Log(string.Format("version {0}.{1}.{2}.{3}", version.field_0, version.field_1, version.field_2, version.field_3));
        Debug.Log(string.Format("utf_offset: {0}", ipc.ts_get_utc_offset()));
        ipc.hs_set_marker_state(hl2ss.hs_marker_state.Disable);
        Debug.Log(string.Format("pv status: {0}", ipc.pv_get_subsystem_status()));
        ipc.pv_wait_for_subsystem(false);
        ipc.pv_set_focus(hl2ss.pv_focus_mode.Manual, hl2ss.pv_auto_focus_range.Normal, hl2ss.pv_manual_focus_distance.Infinity, 1000, hl2ss.pv_driver_fallback.Disable);
        ipc.pv_set_video_temporal_denoising(hl2ss.pv_video_temporal_denoising_mode.On);
        ipc.pv_set_white_balance_preset(hl2ss.pv_color_temperature_preset.Auto);
        ipc.pv_set_white_balance_value(hl2ss.pv_white_balance_value.Min);
        ipc.pv_set_exposure(hl2ss.pv_exposure_mode.Auto, hl2ss.pv_exposure_value.Min);
        ipc.pv_set_exposure_priority_video(hl2ss.pv_exposure_priority_video.Enabled);
        ipc.pv_set_iso_speed(hl2ss.pv_iso_speed_mode.Auto, hl2ss.pv_iso_speed_value.Min);
        ipc.pv_set_backlight_compensation(hl2ss.pv_backlight_compensation_state.Enable);
        ipc.pv_set_scene_mode(hl2ss.pv_capture_scene_mode.Auto);
        ipc.ee_set_flat_mode(false);
        ipc.rm_set_eye_selection(false);
        ipc.pv_set_desired_optimization(hl2ss.pv_media_capture_optimization.LatencyThenPower);
        ipc.pv_set_primary_use(hl2ss.pv_capture_use.Video);
        ipc.pv_set_optical_image_stabilization(hl2ss.pv_optical_image_stabilization_mode.On);
        ipc.pv_set_hdr_video(hl2ss.pv_hdr_video_mode.Off);
        ipc.pv_set_regions_of_interest(true, true, true, true, true, hl2ss.pv_region_of_interest_type.Unknown, 100, 0.0f, 0.0f, 1.0f, 1.0f);
        ipc.ee_set_interface_priority(hl2ss.stream_port.PERSONAL_VIDEO, hl2ss.ee_interface_priority.NORMAL);
        ipc.ee_set_quiet_mode(false);

        var points = ipc.rm_map_camera_points(hl2ss.stream_port.RM_VLC_LEFTFRONT, hl2ss.rm_map_camera_point_operation.ImagePointToCameraUnitPlane, new float[4] { 0, 0, 320, 240 }, 2);
        var timestamp = ipc.ts_get_current_time(hl2ss.ts_source.QPC);
        var poses = ipc.rm_get_rignode_world_poses(new ulong[2] { timestamp, timestamp - hl2ss.time_base.HUNDREDS_OF_NANOSECONDS }, 2);

        var image_points = new float[4];
        Marshal.Copy(points.data, image_points, 0, 4);
        Debug.Log(string.Format("{0} {1} {2} {3}", image_points[0], image_points[1], image_points[2], image_points[3]));

        Debug.Log(timestamp);
        var pose0 = Marshal.PtrToStructure<hl2ss.matrix_4x4>(poses.data);
        var pose1 = Marshal.PtrToStructure<hl2ss.matrix_4x4>(IntPtr.Add(poses.data, Marshal.SizeOf<hl2ss.matrix_4x4>()));
        Debug.Log(string.Format("pose0 [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose0.m_00, pose0.m_01, pose0.m_02, pose0.m_03, pose0.m_10, pose0.m_11, pose0.m_12, pose0.m_13, pose0.m_20, pose0.m_21, pose0.m_22, pose0.m_23, pose0.m_30, pose0.m_31, pose0.m_32, pose0.m_33));
        Debug.Log(string.Format("pose1 [{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}]", pose1.m_00, pose1.m_01, pose1.m_02, pose1.m_03, pose1.m_10, pose1.m_11, pose1.m_12, pose1.m_13, pose1.m_20, pose1.m_21, pose1.m_22, pose1.m_23, pose1.m_30, pose1.m_31, pose1.m_32, pose1.m_33));

        ipc.si_set_sampling_delay(0);
        ipc.ee_set_encoder_buffering(false);
        ipc.ee_set_reader_buffering(false);
        ipc.rm_set_loop_control(hl2ss.stream_port.RM_VLC_LEFTFRONT,  true);
        ipc.rm_set_loop_control(hl2ss.stream_port.RM_VLC_LEFTLEFT,   true);
        ipc.rm_set_loop_control(hl2ss.stream_port.RM_VLC_RIGHTFRONT, true);
        ipc.rm_set_loop_control(hl2ss.stream_port.RM_VLC_RIGHTRIGHT, true);

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
