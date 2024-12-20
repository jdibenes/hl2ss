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

# Disable hl2ss popup warnings
# Requires restarting hl2ss on the HoloLens
quiet_mode = False

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

# Media capture optimization
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.mediacaptureoptimization?view=winrt-26100
capture_optimization = hl2ss.PV_MediaCaptureOptimization.LatencyThenQuality

# Capture use
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.captureuse?view=winrt-26100
primary_use = hl2ss.PV_CaptureUse.Video

# Optical image stabilization
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.opticalimagestabilizationmode?view=winrt-26100
image_stabilization = hl2ss.PV_OpticalImageStabilizationMode.On

# HDR video
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.hdrvideomode?view=winrt-26100
hdr_video = hl2ss.PV_HdrVideoMode.Auto

# Region of interest
# https://learn.microsoft.com/en-us/uwp/api/windows.media.devices.regionofinterest?view=winrt-26100
roi_clear = False
roi_set = False
roi_auto_exposure = True
roi_auto_focus = True
roi_bounds_normalized = True
roi_type = hl2ss.PV_RegionOfInterestType.Unknown
roi_weight = 100
roi_x = 0.0
roi_y = 0.0
roi_w = 1.0
roi_h = 1.0

# Set eye selection (Research Mode)
eye_selection = False

#------------------------------------------------------------------------------

client = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
client.open()

version = client.ee_get_application_version()
print(f'Installed version {version[0]}.{version[1]}.{version[2]}.{version[3]}')

# Add this offset to timestamps to convert to utc (Windows FILETIME)
utc_offset = client.ts_get_utc_offset()
print(f'QPC timestamp to UTC offset is {utc_offset} hundreds of nanoseconds')

client.hs_set_marker_state(marker_state)

# PV camera configuration
pv_status = client.pv_get_subsystem_status()
print(f'PV subsystem is {("On" if (pv_status) else "Off")}')

# Ignored if PV subsystem is Off
client.pv_set_focus(focus_mode, auto_focus_range, manual_focus_distance, focus_value, driver_fallback)
client.pv_set_video_temporal_denoising(video_temporal_denoising)
client.pv_set_white_balance_preset(white_balance_preset)
client.pv_set_white_balance_value(white_balance_value)
client.pv_set_exposure(exposure_mode, exposure_value)
client.pv_set_exposure_priority_video(exposure_priority_video)
client.pv_set_iso_speed(iso_speed_mode, iso_speed_value)
client.pv_set_backlight_compensation(backlight_compensation_state)
client.pv_set_scene_mode(scene_mode)
client.pv_set_desired_optimization(capture_optimization)
client.pv_set_primary_use(primary_use)
client.pv_set_optical_image_stabilization(image_stabilization)
client.pv_set_hdr_video(hdr_video)
client.pv_set_regions_of_interest(roi_clear, roi_set, roi_auto_exposure, roi_auto_focus, roi_bounds_normalized, roi_type, roi_weight, roi_x, roi_y, roi_w, roi_h)

client.rm_set_eye_selection(eye_selection)

client.ee_set_flat_mode(flat_mode)
client.ee_set_quiet_mode(quiet_mode)

# Set interface thread priority
# Useful for time sensitive interfaces when CPU/GPU utilization is high (e.g., streaming multiple sensors)
# Time sensitive interfaces:
# RM VLC
# RM DEPTH AHAT
# RM DEPTH LONGTHROW
# MICROPHONE
# EXTENDED AUDIO
client.ee_set_interface_priority(hl2ss.StreamPort.MICROPHONE, hl2ss.InterfacePriority.NORMAL)

client.close()
