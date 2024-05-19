
#pragma once

#include "custom_video_effect.h"
#include <winrt/Windows.Media.Capture.Frames.h>

void PersonalVideo_Initialize();
void PersonalVideo_Cleanup();
void PersonalVideo_RegisterEvent(HANDLE h);
void PersonalVideo_Open(MRCVideoOptions const& options);
void PersonalVideo_Close();
bool PersonalVideo_Status();
bool PersonalVideo_SetFormat(uint16_t& width, uint16_t& height, uint8_t& framerate);
winrt::Windows::Media::Capture::Frames::MediaFrameReader PersonalVideo_CreateFrameReader();

void PersonalVideo_SetFocus(uint32_t focusmode, uint32_t autofocusrange, uint32_t distance, uint32_t value, uint32_t disabledriverfallback);
void PersonalVideo_SetVideoTemporalDenoising(uint32_t mode);
void PersonalVideo_SetWhiteBalance_Preset(uint32_t preset);
void PersonalVideo_SetWhiteBalance_Value(uint32_t value);
void PersonalVideo_SetExposure(uint32_t setauto, uint32_t value);
void PersonalVideo_SetExposurePriorityVideo(uint32_t enabled);
void PersonalVideo_SetSceneMode(uint32_t mode);
void PersonalVideo_SetIsoSpeed(uint32_t setauto, uint32_t value);
void PersonalVideo_SetBacklightCompensation(bool enable);
