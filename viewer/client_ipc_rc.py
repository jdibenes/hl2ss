#------------------------------------------------------------------------------
# This script shows the available query and configuration options.
# PV camera configurations 1) are ignored while the PV subsystem is stopped,
# 2) can be set even when the PV stream is active, 3) are reset when the PV
# subsystem is restarted.
#------------------------------------------------------------------------------

import hl2ss
import hl2ss_lnm

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Run hl2ss in flat mode
# Allows running hl2ss alongside other applications (including holographics)
# Requires restarting hl2ss on the HoloLens
# Spatial Input not supported in flat mode
flat_mode = False

# Display marker state
# Marks the FOV of the PV camera in the display
marker_state = hl2ss.HS_MarkerState.Disable

# Focus settings
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.focuscontrol?view=winrt-22621
# Presets:
# automatic - FocusMode.Continuous, AutoFocusRange.Normal, ManualFocusDistance.Infinity, 1000,        DriverFallback.Enable
# manual    - FocusMode.Manual,     AutoFocusRange.Normal, ManualFocusDistance.Infinity, focus_value, DriverFallback.Disable
focus_mode = hl2ss.PV_FocusMode.Manual
auto_focus_range = hl2ss.PV_AutoFocusRange.Normal
manual_focus_distance = hl2ss.PV_ManualFocusDistance.Infinity
focus_value = 1000 # 170 - 10000
driver_fallback = hl2ss.PV_DriverFallback.Disable

# Video temporal denoising state
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.videotemporaldenoisingcontrol?view=winrt-22621
video_temporal_denoising = hl2ss.PV_VideoTemporalDenoisingMode.Off

# White balance settings
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.whitebalancecontrol?view=winrt-22621
white_balance_preset = hl2ss.PV_ColorTemperaturePreset.Cloudy
white_balance_value = hl2ss.PV_WhiteBalanceValue.Min # 92 - 300

# Exposure mode
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.exposurecontrol?view=winrt-22621
exposure_mode = hl2ss.PV_ExposureMode.Manual
exposure_value = 10000 # 100 - 66000

# Exposure priority video
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.exposurepriorityvideocontrol?view=winrt-22621
exposure_priority_video = hl2ss.PV_ExposurePriorityVideo.Disabled

# ISO speed
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.isospeedcontrol?view=winrt-22621
iso_speed_mode = hl2ss.PV_IsoSpeedMode.Auto
iso_speed_value = hl2ss.PV_IsoSpeedValue.Min # 100-3200

# Backlight compensation
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.videodevicecontroller.backlightcompensation?view=winrt-22621#windows-media-devices-videodevicecontroller-backlightcompensation
backlight_compensation_state = hl2ss.PV_BacklightCompensationState.Enable

# Scene mode
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.scenemodecontrol?view=winrt-22621
scene_mode = hl2ss.PV_CaptureSceneMode.Auto

# Set eye selection
eye_selection = False

#------------------------------------------------------------------------------

client = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
client.open()

version = client.get_application_version()
print(f'Installed version {version[0]}.{version[1]}.{version[2]}.{version[3]}')

# Add this offset to timestamps to convert to utc (Windows FILETIME)
utc_offset = client.get_utc_offset(32)
print(f'QPC timestamp to UTC offset is {utc_offset} hundreds of nanoseconds')

client.set_hs_marker_state(marker_state)

# PV camera configuration
pv_status = client.get_pv_subsystem_status()
print(f'PV subsystem is {("On" if (pv_status) else "Off")}')

# Ignored if PV subsystem is Off
client.set_pv_focus(focus_mode, auto_focus_range, manual_focus_distance, focus_value, driver_fallback)
client.set_pv_video_temporal_denoising(video_temporal_denoising)
client.set_pv_white_balance_preset(white_balance_preset)
client.set_pv_white_balance_value(white_balance_value)
client.set_pv_exposure(exposure_mode, exposure_value)
client.set_pv_exposure_priority_video(exposure_priority_video)
client.set_pv_iso_speed(iso_speed_mode, iso_speed_value)
client.set_pv_backlight_compensation(backlight_compensation_state)
client.set_pv_scene_mode(scene_mode)

client.set_flat_mode(flat_mode)

client.set_rm_eye_selection(eye_selection)

client.close()
