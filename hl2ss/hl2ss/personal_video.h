
#pragma once

#include <winrt/Windows.Media.Capture.Frames.h>

void PersonalVideo_Initialize();
bool PersonalVideo_SetFormat(uint32_t width, uint32_t height, uint32_t framerate);
winrt::Windows::Media::Capture::Frames::MediaFrameReader PersonalVideo_CreateFrameReader();
void PersonalVideo_SetFocus(uint32_t focusmode, uint32_t autofocusrange, uint32_t distance, uint32_t value, uint32_t disabledriverfallback);
void PersonalVideo_SetVideoTemporalDenoising(uint32_t mode);
void PersonalVideo_SetWhiteBalance_Preset(uint32_t preset);
void PersonalVideo_SetWhiteBalance_Value(uint32_t value);
void PersonalVideo_SetExposure(uint32_t setauto, uint32_t value);
