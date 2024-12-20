
#pragma once

#include <Windows.h>
#include "custom_media_types.h"
#include "custom_video_effect.h"

#include <winrt/Windows.Media.Capture.Frames.h>

typedef void (*HOOK_EV_PROC)(winrt::Windows::Media::Capture::Frames::MediaFrameReference const&, void* param);

void ExtendedVideo_CreateNamedMutex(wchar_t const* name);

winrt::hstring ExtendedVideo_QueryDevices();
void ExtendedVideo_Open(MRCVideoOptions const& options);
void ExtendedVideo_Close();
bool ExtendedVideo_Status();
bool ExtendedVideo_SetFormat(uint16_t& width, uint16_t& height, uint8_t& framerate, VideoSubtype& subtype);
void ExtendedVideo_ExecuteSensorLoop(winrt::Windows::Media::Capture::Frames::MediaFrameReaderAcquisitionMode mode, HOOK_EV_PROC hook, void* param, HANDLE event_stop);
