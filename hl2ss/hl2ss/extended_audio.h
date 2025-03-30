
#pragma once

#include "custom_media_types.h"
#include "custom_audio_effect.h"

#include <winrt/Windows.Media.Capture.Frames.h>

struct ExtendedAudio_Control
{
    uint32_t device_index;
    uint32_t source_index;
    uint32_t format_index;
    uint32_t media_category;
    bool shared;
    bool _reserved_0;
    bool audio_raw;
    bool disable_effect;
    bool enable_passthrough;
    bool query;
    uint16_t _reserved_1;
};

typedef void (*HOOK_EA_PROC)(winrt::Windows::Media::Capture::Frames::MediaFrameReference const&, void*);

winrt::hstring ExtendedAudio_QueryDevices(MRCAudioOptions const& options, ExtendedAudio_Control const& control);
void ExtendedAudio_Open(MRCAudioOptions const& options, ExtendedAudio_Control const& control);
void ExtendedAudio_Close();
bool ExtendedAudio_Status();
void ExtendedAudio_GetCurrentFormat(AudioSubtype& subtype, uint32_t& channels, uint32_t& samplerate);
void ExtendedAudio_ExecuteSensorLoop(HOOK_EA_PROC hook, void* param, HANDLE event_stop);
