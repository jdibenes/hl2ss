
#pragma once

#include "custom_media_sink.h"
#include "custom_media_types.h"
#include "custom_hook_callback.h"
#include <mfreadwrite.h>
#include <vector>

struct H26xFormat
{
    uint16_t    width;
    uint16_t    height;
    uint8_t     framerate;
    uint8_t     divisor;
    H26xProfile profile;
    int8_t      level;
    uint32_t    bitrate;
};

struct AACFormat
{
    uint16_t   samplerate;
    uint8_t    channels;    
    AACProfile profile;
    AACLevel   level;
    uint8_t    _reserved[3];
};

struct HookCallbackSocket
{
    SOCKET clientsocket;
    HANDLE clientevent;
    void*  format;
};

void CreateSinkWriterAudio(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, AudioSubtype input_subtype, AACFormat  const& format, HOOK_SINK_PROC hookproc, void* hookparam);
void CreateSinkWriterVideo(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, VideoSubtype input_subtype, H26xFormat const& format, std::vector<uint64_t> const& encoder_options, HOOK_SINK_PROC hookproc, void* hookparam);
