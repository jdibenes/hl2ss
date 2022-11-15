#------------------------------------------------------------------------------
# This script configures the HoloLens front RGB camera. Parameters can be set
# even while the PV stream is active. Parameters are reset when the server
# application is closed.
#------------------------------------------------------------------------------

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Display marker state
marker_state = hl2ss.MarkerState.Disable

# Focus Mode
# Presets:
# automatic - FocusMode.Continuous, AutoFocusRange.Normal, ManualFocusDistance.Infinity, 1000,        DriverFallback.Enable
# manual    - FocusMode.Manual,     AutoFocusRange.Normal, ManualFocusDistance.Infinity, focus_value, DriverFallback.Disable
focus_mode = hl2ss.FocusMode.Manual

# Ignored for FocusMode.Manual
auto_focus_range = hl2ss.AutoFocusRange.Normal

# For FocusMode.Manual only
manual_focus_distance = hl2ss.ManualFocusDistance.Infinity

# Focus value - only used when FocusMode.Manual is set
# 170 - 10000
focus_value = 1000

# Ignored for FocusMode.Manual
driver_fallback = hl2ss.DriverFallback.Disable

# Video temporal denoising state
video_temporal_denoising = hl2ss.VideoTemporalDenoisingMode.Off

# White balance preset
white_balance_preset = hl2ss.ColorTemperaturePreset.Cloudy

# Exposure mode
exposure_mode = hl2ss.ExposureMode.Manual

# Exposure value - ignored when ExposureMode.Auto is set
# 100 - 66000
exposure_value = 10000

#------------------------------------------------------------------------------

client = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)

version = client.get_application_version()
print('Installed HL2SS version {v0}.{v1}.{v2}.{v3}'.format(v0=version[0], v1=version[1], v2=version[2], v3=version[3]))

utc_offset = client.get_utc_offset(32)
print('QPC timestamp to UTC offset is {offset} hns'.format(offset=utc_offset))

pv_status = client.get_pv_subsystem_status()
print('PV subsystem status is {status}'.format(status=('On' if pv_status else 'Off')))

client.set_marker_state(marker_state)
client.set_pv_focus(focus_mode, auto_focus_range, manual_focus_distance, focus_value, driver_fallback)
client.set_pv_video_temporal_denoising(video_temporal_denoising)
client.set_pv_white_balance_preset(white_balance_preset)
client.set_pv_white_balance_value(hl2ss.WhiteBalanceValue.Min)
client.set_pv_exposure(exposure_mode, exposure_value)
client.set_pv_exposure_priority_video(hl2ss.ExposurePriorityVideo.Disabled)
client.set_pv_iso_speed(hl2ss.IsoSpeedMode.Auto, hl2ss.IsoSpeedValue.Min)
client.set_pv_scene_mode(hl2ss.CaptureSceneMode.Auto)
