
#pragma once

#include "custom_media_types.h"
#include "custom_audio_effect.h"

#include <winrt/Windows.Media.Capture.Frames.h>

typedef void (*HOOK_EA_PROC)(winrt::Windows::Media::Capture::Frames::MediaFrameReference const&, void*);

winrt::hstring ExtendedAudio_QueryDevices();
void ExtendedAudio_Open(MRCAudioOptions const& options);
void ExtendedAudio_Close();
bool ExtendedAudio_Status();
void ExtendedAudio_GetCurrentFormat(AudioSubtype& subtype, uint32_t& channels);
void ExtendedAudio_ExecuteSensorLoop(HOOK_EA_PROC hook, void* param, HANDLE event_stop);
