
#pragma once

#include <Windows.h>
#include "custom_video_effect.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Media.Capture.Frames.h>

typedef void (*HOOK_PV_PROC)(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame, void* param);

void PersonalVideo_CreateNamedMutex(wchar_t const* name);

void PersonalVideo_Startup();
void PersonalVideo_Cleanup();
void PersonalVideo_Open(MRCVideoOptions const& options);
void PersonalVideo_Close();
bool PersonalVideo_Status();
uint32_t PersonalVideo_GetStride(uint32_t width);
winrt::Windows::Foundation::Numerics::float4x4 PersonalVideo_GetFrameWorldPose(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame);
bool PersonalVideo_SetFormat(uint16_t& width, uint16_t& height, uint8_t& framerate);
void PersonalVideo_ExecuteSensorLoop(winrt::Windows::Media::Capture::Frames::MediaFrameReaderAcquisitionMode mode, HOOK_PV_PROC hook, void* param, HANDLE event_stop);

void PersonalVideo_SetFocus(uint32_t focusmode, uint32_t autofocusrange, uint32_t distance, uint32_t value, uint32_t disabledriverfallback);
void PersonalVideo_SetVideoTemporalDenoising(uint32_t mode);
void PersonalVideo_SetWhiteBalance_Preset(uint32_t preset);
void PersonalVideo_SetWhiteBalance_Value(uint32_t value);
void PersonalVideo_SetExposure(uint32_t setauto, uint32_t value);
void PersonalVideo_SetExposurePriorityVideo(uint32_t enabled);
void PersonalVideo_SetSceneMode(uint32_t mode);
void PersonalVideo_SetIsoSpeed(uint32_t setauto, uint32_t value);
void PersonalVideo_SetBacklightCompensation(bool enable);
void PersonalVideo_SetDesiredOptimization(uint32_t mode);
void PersonalVideo_SetPrimaryUse(uint32_t mode);
void PersonalVideo_SetOpticalImageStabilization(uint32_t mode);
void PersonalVideo_SetHdrVideo(uint32_t mode);
void PersonalVideo_SetRegionsOfInterest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, float x, float y, float w, float h, uint32_t type, uint32_t weight);
