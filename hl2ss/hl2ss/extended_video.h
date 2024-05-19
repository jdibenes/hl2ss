
#pragma once

#include "custom_media_types.h"
#include "custom_video_effect.h"
#include <winrt/Windows.Media.Capture.Frames.h>

void ExtendedVideo_QueryDevices(winrt::hstring& out);
void ExtendedVideo_RegisterEvent(HANDLE h);
void ExtendedVideo_Open(MRCVideoOptions const& options);
void ExtendedVideo_Close();
bool ExtendedVideo_Status();
bool ExtendedVideo_SetFormat(uint16_t& width, uint16_t& height, uint8_t& framerate, VideoSubtype& subtype);
winrt::Windows::Media::Capture::Frames::MediaFrameReader ExtendedVideo_CreateFrameReader();
