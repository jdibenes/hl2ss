%%
% This script shows the available query and configuration options.

%% Settings

% HoloLens address
host = '192.168.1.7';

%%

client = hl2ss.mt.ipc_rc(host, hl2ss.ipc_port.REMOTE_CONFIGURATION);
client.open();

try
version    = client.get_application_version();
pv_status  = client.get_pv_subsystem_status();
utc_offset = client.get_utc_offset();

client.set_hs_marker_state(hl2ss.hs_marker_state.Disable);
client.set_pv_backlight_compensation(hl2ss.pv_backlight_compensation_state.Disable);
client.set_pv_exposure(hl2ss.pv_exposure_mode.Auto, hl2ss.pv_exposure_value.Min);
client.set_pv_exposure_priority_video(hl2ss.pv_exposure_priority_video.Disabled);
client.set_pv_focus(hl2ss.pv_focus_mode.Manual, hl2ss.pv_auto_focus_range.Normal, hl2ss.pv_manual_focus_distance.Infinity, hl2ss.pv_focus_value.Min, hl2ss.pv_driver_fallback.Disable);
client.set_pv_iso_speed(hl2ss.pv_iso_speed_mode.Auto, hl2ss.pv_iso_speed_value.Min);
client.set_pv_scene_mode(hl2ss.pv_capture_scene_mode.Auto);
client.set_pv_video_temporal_denoising(hl2ss.pv_video_temporal_denoising_mode.Off);
client.set_pv_white_balance_preset(hl2ss.pv_color_temperature_preset.Auto);
client.set_pv_white_balance_value(hl2ss.pv_white_balance_value.Min);
client.set_flat_mode(false);
client.set_rm_eye_selection(false);
client.set_pv_desired_optimization(hl2ss.pv_media_capture_optimization.LatencyThenPower);
client.set_pv_primary_use(hl2ss.pv_capture_use.Video);
client.set_pv_optical_image_stabilization(hl2ss.pv_optical_stabilization_mode.On);
client.set_pv_hdr_video(hl2ss.pv_hdr_video_mode.Off);
client.set_pv_regions_of_interest(true, true, true, true, true, hl2ss.pv_region_of_interest_type.Unknown, 100, 0, 0, 1, 1);
client.set_interface_priority(hl2ss.stream_port.PERSONAL_VIDEO, hl2ss.interface_priority.ABOVE_NORMAL);
client.set_quiet_mode(false);
catch ME
    disp(ME.message);
end

client.close();
