#------------------------------------------------------------------------------
# This script shows available query and configuration options with examples.
#------------------------------------------------------------------------------

from datetime import datetime

import numpy as np
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
# Not supported by the plugin
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
iso_speed_value = hl2ss.PV_IsoSpeedValue.Min # 100 - 3200

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
roi_weight = 100 # 0 - 100
roi_x = 0.0
roi_y = 0.0
roi_w = 1.0
roi_h = 1.0

# Set eye selection (Research Mode)
# no idea what it does
eye_selection = False

# Interface priorities
# Useful for time sensitive interfaces when CPU/GPU utilization is high 
# e.g., when streaming multiple sensors
# Time sensitive interfaces:
# RM VLC
# RM DEPTH AHAT
# RM DEPTH LONGTHROW
# MICROPHONE
interface_priority = {
    # Streams
    hl2ss.StreamPort.RM_VLC_LEFTFRONT     : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT      : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_DEPTH_AHAT        : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW   : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_IMU_ACCELEROMETER : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_IMU_GYROSCOPE     : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.RM_IMU_MAGNETOMETER  : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.PERSONAL_VIDEO       : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.MICROPHONE           : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.SPATIAL_INPUT        : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.EXTENDED_EYE_TRACKER : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.EXTENDED_AUDIO       : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.EXTENDED_VIDEO       : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.StreamPort.EXTENDED_DEPTH       : hl2ss.EE_InterfacePriority.NORMAL,
    # IPCs
    hl2ss.IPCPort.REMOTE_CONFIGURATION : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.IPCPort.SPATIAL_MAPPING      : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.IPCPort.SCENE_UNDERSTANDING  : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.IPCPort.VOICE_INPUT          : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.IPCPort.UNITY_MESSAGE_QUEUE  : hl2ss.EE_InterfacePriority.NORMAL,
    hl2ss.IPCPort.GUEST_MESSAGE_QUEUE  : hl2ss.EE_InterfacePriority.NORMAL,
}

# Spatial Input sampling delay
# Delay SI readouts by the specified amount (in hundreds of nanoseconds)
si_sampling_delay = 0

# Encoder buffering
# Buffer encoded frames before sending over network
encoder_buffering = False

# Reader buffering
# Buffer sensor frames before sending to the encoder
reader_buffering = False

# RM VLC loop control
# Set to True to enable workaround for RM VLC issues
# Might crash on future HoloLens OS / Research Mode DLL versions
rm_vlc_loop_control = False

#------------------------------------------------------------------------------

# Connect to server -----------------------------------------------------------
client = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
client.open()

# Get installed version -------------------------------------------------------

print('Package -----------------------------------------------------------------------')

version = client.ee_get_application_version() # Note that the plugin returns main application version not hl2ss version
print(f'Installed version {version[0]}.{version[1]}.{version[2]}.{version[3]}')

# Timestamp helpers -----------------------------------------------------------
# Make sure your HoloLens and PC timezone is set correctly
# hl2ss Stream timestamps are given in QPC domain
# hl2ss IPC    timestamps are given in UTC domain

print('Timing ------------------------------------------------------------------------')

utc_offset = client.ts_get_utc_offset() # Add this offset to stream timestamps to convert to UTC (Windows FILETIME)

# Get current time (network delay is not considered)
hl2_qpc_time = client.ts_get_current_time(hl2ss.TS_Source.QPC) # Get HoloLens 2 current time in QPC domain
pc_datetime = datetime.now() # Get current time
hl2_wft_time = client.ts_get_current_time(hl2ss.TS_Source.UTC) # Get HoloLens 2 current time in UTC domain

# Conversion from QPC to datetime
hl2_utc_time = hl2ss.ts_qpc_to_filetime(hl2_qpc_time, utc_offset) # Convert from QPC to UTC (Windows FILETIME)
hl2_posix_hns_time = hl2ss.ts_filetime_to_unix_hns(hl2_utc_time) # Convert from UTC (Windows FILETIME) to Unix time (in hundreds of nanoseconds)
hl2_posix_time = hl2ss.ts_unix_hns_to_unix(hl2_posix_hns_time) # Convert Unix time (in hundreds of nanoseconds) to Unix time (in seconds)
hl2_qpc_datetime = datetime.fromtimestamp(hl2_posix_time) # Convert Unix time to datetime

# Conversion from UTC to datetime
hl2_posix_hns_time = hl2ss.ts_filetime_to_unix_hns(hl2_wft_time)
hl2_posix_time = hl2ss.ts_unix_hns_to_unix(hl2_posix_hns_time)
hl2_wft_datetime = datetime.fromtimestamp(hl2_posix_time)

# Conversion from datetime to QPC
pc_posix_time = pc_datetime.timestamp()
pc_posix_hns_time = hl2ss.ts_unix_to_unix_hns(pc_posix_time)
pc_utc_time = hl2ss.ts_unix_hns_to_filetime(pc_posix_hns_time)
pc_qpc_time = hl2ss.ts_filetime_to_qpc(pc_utc_time, utc_offset)

print(f'HL2 QPC to UTC offset    : {utc_offset} hundreds of nanoseconds')
print(f'DateTime HL2 (QPC)       : {hl2_qpc_datetime}')
print(f'DateTime HL2 (UTC)       : {hl2_wft_datetime}')
print(f'DateTime This PC         : {pc_datetime}')
print(f'QPC timestamp HL2        : {hl2_qpc_time}')
print(f'QPC timestamp This PC    : {pc_qpc_time}')
print(f'QPC offset This PC - HL2 : {(pc_qpc_time - hl2_qpc_time) / hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS} seconds')

# PV camera configuration -----------------------------------------------------
# PV camera configurations
# 1) are ignored while the PV subsystem is stopped
# 2) can be set even when the PV stream is active
# 3) are reset when the PV subsystem is restarted

print('PV camera ---------------------------------------------------------------------')

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

# Auxiliary Research Mode methods ---------------------------------------------

print('Research Mode -----------------------------------------------------------------')

client.rm_set_eye_selection(eye_selection) # Set eye selection
print(f'Set Eye Selection : {eye_selection}')

# Image-Camera space conversions
# Input is a float32 numpy array of any shape but
# 1) the number of elements must be a multiple of 2
# 2) points must be stored in memory as x0,y0,x1,y1,...,xn,yn

# Convert the four image corners and center of image to normalized coordinates
image_corners_vlc = np.array([[ 0,  0], [hl2ss.Parameters_RM_VLC.WIDTH             -  1,  0], [hl2ss.Parameters_RM_VLC.WIDTH             / 2, hl2ss.Parameters_RM_VLC.HEIGHT             / 2], [ 0, hl2ss.Parameters_RM_VLC.HEIGHT             -  1], [hl2ss.Parameters_RM_VLC.WIDTH             -  1, hl2ss.Parameters_RM_VLC.HEIGHT             -  1]], dtype=np.float32)
image_corners_zht = np.array([[80, 80], [hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH      - 81, 80], [hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH      / 2, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT      / 2], [80, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT      - 81], [hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH      - 81, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT      - 81]], dtype=np.float32)
image_corners_zlt = np.array([[ 0,  0], [hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH -  1,  0], [hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH / 2, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT / 2], [ 0, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT -  1], [hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH -  1, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT -  1]], dtype=np.float32)

normalized_corners_vlf = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_LEFTFRONT,   hl2ss.RM_MapCameraPointOperation.ImagePointToCameraUnitPlane, image_corners_vlc)
normalized_corners_vll = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_LEFTLEFT,    hl2ss.RM_MapCameraPointOperation.ImagePointToCameraUnitPlane, image_corners_vlc)
normalized_corners_vrf = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_RIGHTFRONT,  hl2ss.RM_MapCameraPointOperation.ImagePointToCameraUnitPlane, image_corners_vlc)
normalized_corners_vrr = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,  hl2ss.RM_MapCameraPointOperation.ImagePointToCameraUnitPlane, image_corners_vlc)
normalized_corners_zht = client.rm_map_camera_points(hl2ss.StreamPort.RM_DEPTH_AHAT,      hl2ss.RM_MapCameraPointOperation.ImagePointToCameraUnitPlane, image_corners_zht)
normalized_corners_zlt = client.rm_map_camera_points(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.RM_MapCameraPointOperation.ImagePointToCameraUnitPlane, image_corners_zlt)

# Convert back to image coordinates
# For RM Depth AHAT, the CameraSpaceToImagePoint operation does not cover the entire image
imaged_corners_vlf = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_LEFTFRONT,   hl2ss.RM_MapCameraPointOperation.CameraSpaceToImagePoint, normalized_corners_vlf)
imaged_corners_vll = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_LEFTLEFT,    hl2ss.RM_MapCameraPointOperation.CameraSpaceToImagePoint, normalized_corners_vll)
imaged_corners_vrf = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_RIGHTFRONT,  hl2ss.RM_MapCameraPointOperation.CameraSpaceToImagePoint, normalized_corners_vrf)
imaged_corners_vrr = client.rm_map_camera_points(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,  hl2ss.RM_MapCameraPointOperation.CameraSpaceToImagePoint, normalized_corners_vrr)
imaged_corners_zht = client.rm_map_camera_points(hl2ss.StreamPort.RM_DEPTH_AHAT,      hl2ss.RM_MapCameraPointOperation.CameraSpaceToImagePoint, normalized_corners_zht)
imaged_corners_zlt = client.rm_map_camera_points(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.RM_MapCameraPointOperation.CameraSpaceToImagePoint, normalized_corners_zlt)

print(f'RM VLF transform loss {np.linalg.norm((image_corners_vlc - imaged_corners_vlf).reshape((-1)))}')
print(f'RM VLL transform loss {np.linalg.norm((image_corners_vlc - imaged_corners_vll).reshape((-1)))}')
print(f'RM VRF transform loss {np.linalg.norm((image_corners_vlc - imaged_corners_vrf).reshape((-1)))}')
print(f'RM VRR transform loss {np.linalg.norm((image_corners_vlc - imaged_corners_vrr).reshape((-1)))}')
print(f'RM ZHT transform loss {np.linalg.norm((image_corners_zht - imaged_corners_zht).reshape((-1)))}')
print(f'RM ZLT transform loss {np.linalg.norm((image_corners_zlt - imaged_corners_zlt).reshape((-1)))}')

# RigNode pose query
# Input is a uint64 numpy array of any shape containing timestamps in QPC domain
hl2_qpc_time = client.ts_get_current_time(hl2ss.TS_Source.QPC)
pose_timestamps = np.array([hl2_qpc_time], dtype=np.uint64)
poses = client.rm_get_rignode_world_poses(pose_timestamps)

print(f'RigNode pose at (QPC) time {hl2_qpc_time}')
print(poses)

# System control --------------------------------------------------------------

print('System control -----------------------------------------------------------------')

client.hs_set_marker_state(marker_state) # Display PV field-of-view marker on screen
print(f'Set PV marker state : {marker_state}')

client.ee_set_flat_mode(flat_mode) # Set flat mode
print(f'Set flat mode : {flat_mode}')

client.ee_set_quiet_mode(quiet_mode) # Set quiet mode
print(f'Set quiet mode : {quiet_mode}')

# Set interface thread priority
for port, priority in interface_priority.items():
    client.ee_set_interface_priority(port, priority)
    print(f'Set interface {hl2ss.get_port_name(port)} priority : {priority}')

client.si_set_sampling_delay(si_sampling_delay) # Set Spatial Input sampling delay
print(f'Set SI sampling delay : {si_sampling_delay}')

client.ee_set_encoder_buffering(encoder_buffering)
print(f'Set encoder buffering : {encoder_buffering}')

client.ee_set_reader_buffering(reader_buffering)
print(f'Set reader buffering : {reader_buffering}')

client.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_LEFTFRONT,  rm_vlc_loop_control)
client.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_LEFTLEFT,   rm_vlc_loop_control)
client.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, rm_vlc_loop_control)
client.rm_set_loop_control(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, rm_vlc_loop_control)
print(f'Set rm vlc loop control : {rm_vlc_loop_control}')

# Disconnect ------------------------------------------------------------------
client.close()
