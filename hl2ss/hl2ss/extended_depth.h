
#pragma once

#include <Windows.h>
#include "custom_video_effect.h"

#include <winrt/Windows.Media.Capture.Frames.h>

typedef void (*HOOK_EZ_PROC)(winrt::Windows::Media::Capture::Frames::MediaFrameReference const&, void* param);

void ExtendedDepth_Open(MRCVideoOptions const& options);
void ExtendedDepth_Close();
bool ExtendedDepth_Status();
bool ExtendedDepth_SetFormat(uint32_t index, uint16_t& width, uint16_t& height, uint8_t& framerate, winrt::hstring& subtype);
void ExtendedDepth_ExecuteSensorLoop(winrt::Windows::Media::Capture::Frames::MediaFrameReaderAcquisitionMode mode, HOOK_EZ_PROC hook, void* param, HANDLE event_stop);
