
#pragma once

#include "custom_media_types.h"
#include "custom_audio_effect.h"
#include <winrt/Windows.Media.Capture.Frames.h>

void ExtendedAudio_QueryDevices(winrt::hstring& out);
void ExtendedAudio_RegisterEvent(HANDLE h);
bool ExtendedAudio_Open(MRCAudioOptions const& options);
void ExtendedAudio_Close();
winrt::Windows::Media::Capture::Frames::MediaFrameReader ExtendedAudio_CreateFrameReader();
