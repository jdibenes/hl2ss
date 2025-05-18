%%
% This script shows the available query and configuration options.

%% Settings

% HoloLens address
host = '192.168.1.7';

%%

client = hl2ss.mt.ipc_rc(host, hl2ss.ipc_port.REMOTE_CONFIGURATION);
client.open();

try
version = client.ee_get_application_version();
utc_offset = client.ts_get_utc_offset();
client.hs_set_marker_state(hl2ss.hs_marker_state.Disable);
pv_status = client.pv_get_subsystem_status();
client.pv_set_focus(hl2ss.pv_focus_mode.Manual, hl2ss.pv_auto_focus_range.Normal, hl2ss.pv_manual_focus_distance.Infinity, hl2ss.pv_focus_value.Min, hl2ss.pv_driver_fallback.Disable);
client.pv_set_video_temporal_denoising(hl2ss.pv_video_temporal_denoising_mode.Off);
client.pv_set_white_balance_preset(hl2ss.pv_color_temperature_preset.Auto);
client.pv_set_white_balance_value(hl2ss.pv_white_balance_value.Min);
client.pv_set_exposure(hl2ss.pv_exposure_mode.Auto, hl2ss.pv_exposure_value.Min);
client.pv_set_exposure_priority_video(hl2ss.pv_exposure_priority_video.Disabled);
client.pv_set_iso_speed(hl2ss.pv_iso_speed_mode.Auto, hl2ss.pv_iso_speed_value.Min);
client.pv_set_backlight_compensation(hl2ss.pv_backlight_compensation_state.Disable);
client.pv_set_scene_mode(hl2ss.pv_capture_scene_mode.Auto);
client.ee_set_flat_mode(false);
client.rm_set_eye_selection(false);
client.pv_set_desired_optimization(hl2ss.pv_media_capture_optimization.LatencyThenPower);
client.pv_set_primary_use(hl2ss.pv_capture_use.Video);
client.pv_set_optical_image_stabilization(hl2ss.pv_optical_stabilization_mode.On);
client.pv_set_hdr_video(hl2ss.pv_hdr_video_mode.Off);
client.pv_set_regions_of_interest(true, true, true, true, true, hl2ss.pv_region_of_interest_type.Unknown, 100, 0, 0, 1, 1);
client.ee_set_interface_priority(hl2ss.stream_port.PERSONAL_VIDEO, hl2ss.ee_interface_priority.ABOVE_NORMAL);
client.ee_set_quiet_mode(false);
image_points = client.rm_map_camera_points(hl2ss.stream_port.RM_VLC_LEFTFRONT, hl2ss.rm_map_camera_point_operation.ImagePointToCameraUnitPlane, [0, 0, 320, 240]);
qpc_time = client.ts_get_current_time(hl2ss.ts_source.QPC);
poses = client.rm_get_rignode_world_poses([qpc_time, qpc_time - hl2ss.time_base.HUNDREDS_OF_NANOSECONDS]); % TRANSPOSED
client.si_set_sampling_delay(0);
client.ee_set_encoder_buffering(false);
client.ee_set_reader_buffering(false);
client.rm_set_loop_control(hl2ss.stream_port.RM_VLC_LEFTFRONT,  true);
client.rm_set_loop_control(hl2ss.stream_port.RM_VLC_LEFTLEFT,   true);
client.rm_set_loop_control(hl2ss.stream_port.RM_VLC_RIGHTFRONT, true);
client.rm_set_loop_control(hl2ss.stream_port.RM_VLC_RIGHTRIGHT, true);
catch ME
    disp(ME.message);
end

client.close();
