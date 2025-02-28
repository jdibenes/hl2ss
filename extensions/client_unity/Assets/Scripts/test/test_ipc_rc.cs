
using UnityEngine;

public class test_ipc_rc : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string host = run_once.host_address;

        hl2ss.svc.open_ipc(host, hl2ss.ipc_port.REMOTE_CONFIGURATION, out hl2ss.svc.ipc_rc ipc);

        hl2ss.version version = ipc.get_application_version();
        Debug.Log(string.Format("version {0}.{1}.{2}.{3}", version.field_0, version.field_1, version.field_2, version.field_3));
        Debug.Log(string.Format("utf_offset: {0}", ipc.get_utc_offset()));
        ipc.set_hs_marker_state(hl2ss.hs_marker_state.Disable);
        Debug.Log(string.Format("pv status: {0}", ipc.get_pv_subsystem_status()));
        ipc.wait_for_pv_subsystem(false);
        ipc.set_pv_focus(hl2ss.pv_focus_mode.Manual, hl2ss.pv_auto_focus_range.Normal, hl2ss.pv_manual_focus_distance.Infinity, 1000, hl2ss.pv_driver_fallback.Disable);
        ipc.set_pv_video_temporal_denoising(hl2ss.pv_video_temporal_denoising_mode.On);
        ipc.set_pv_white_balance_preset(hl2ss.pv_color_temperature_preset.Auto);
        ipc.set_pv_white_balance_value(hl2ss.pv_white_balance_value.Min);
        ipc.set_pv_exposure(hl2ss.pv_exposure_mode.Auto, hl2ss.pv_exposure_value.Min);
        ipc.set_pv_exposure_priority_video(hl2ss.pv_exposure_priority_video.Enabled);
        ipc.set_pv_iso_speed(hl2ss.pv_iso_speed_mode.Auto, hl2ss.pv_iso_speed_value.Min);
        ipc.set_pv_backlight_compensation(hl2ss.pv_backlight_compensation_state.Enable);
        ipc.set_pv_scene_mode(hl2ss.pv_capture_scene_mode.Auto);
        ipc.set_flat_mode(false);
        ipc.set_rm_eye_selection(false);
        ipc.set_pv_desired_optimization(hl2ss.pv_media_capture_optimization.LatencyThenPower);
        ipc.set_pv_primary_use(hl2ss.pv_capture_use.Video);
        ipc.set_pv_optical_image_stabilization(hl2ss.pv_optical_image_stabilization_mode.On);
        ipc.set_pv_hdr_video(hl2ss.pv_hdr_video_mode.Off);
        ipc.set_pv_regions_of_interest(true, true, true, true, true, hl2ss.pv_region_of_interest_type.Unknown, 100, 0.0f, 0.0f, 1.0f, 1.0f);
        ipc.set_interface_priority(hl2ss.stream_port.PERSONAL_VIDEO, hl2ss.interface_priority.ABOVE_NORMAL);
        ipc.set_quiet_mode(false);

        ipc.Dispose();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
