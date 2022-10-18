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
marker_state = hl2ss.MarkerState.Enable

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

version = client.get_version()
print(version)

utc_offset = client.get_utc_offset()
print(utc_offset)

quit()

client.set_marker_state(marker_state)
client.set_focus(focus_mode, auto_focus_range, manual_focus_distance, focus_value, driver_fallback)
client.set_video_temporal_denoising(video_temporal_denoising)
client.set_white_balance_preset(white_balance_preset)
client.set_exposure(exposure_mode, exposure_value)
