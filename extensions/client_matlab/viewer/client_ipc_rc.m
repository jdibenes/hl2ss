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
utc_offset = client.get_utc_offset(32);

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
client.set_flat_mode(0)

catch ME
    disp(ME.message);
end

client.close();
