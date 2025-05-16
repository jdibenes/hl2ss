
using UnityEngine;

public class test_ipc_rc : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.open_ipc(host, hl2ss.ipc_port.REMOTE_CONFIGURATION, out hl2ss.svc.ipc_rc ipc);

        hl2ss.version version = ipc.ee_get_application_version();
        Debug.Log(string.Format("version {0}.{1}.{2}.{3}", version.field[0], version.field[1], version.field[2], version.field[3]));
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
        ipc.ee_set_interface_priority(hl2ss.stream_port.PERSONAL_VIDEO, hl2ss.ee_interface_priority.ABOVE_NORMAL);
        ipc.ee_set_quiet_mode(false);

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
