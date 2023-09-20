
hl2ss_matlab('open', '192.168.1.7', uint16(3809));

version = hl2ss_matlab('ipc_call', uint16(3809), 'get_application_version');
pv_status = hl2ss_matlab('ipc_call', uint16(3809), 'get_pv_subsystem_status');
utc_offset = hl2ss_matlab('ipc_call', uint16(3809), 'get_utc_offset', uint32(32));

hl2ss_matlab('ipc_call', uint16(3809), 'set_hs_marker_state', uint32(1));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_backlight_compensation', uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_exposure', uint32(0), uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_exposure_priority_video', uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_focus', uint32(0), uint32(0), uint32(0), uint32(0), uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_iso_speed', uint32(0), uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_scene_mode', uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_video_temporal_denoising', uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_white_balance_preset', uint32(0));
hl2ss_matlab('ipc_call', uint16(3809), 'set_pv_white_balance_value', uint32(0));

hl2ss_matlab('close', uint16(3809));
