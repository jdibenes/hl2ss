
#pragma once

#include "custom_audio_effect.h"
#include <winrt/Windows.Media.Capture.Frames.h>

void ExtendedAudio_Open(MRCAudioOptions const& options);
void ExtendedAudio_Close();
winrt::Windows::Media::Capture::Frames::MediaFrameReader ExtendedAudio_CreateFrameReader();
