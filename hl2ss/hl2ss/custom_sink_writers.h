
#pragma once

#include "custom_media_types.h"
#include "custom_hook_callback.h"
#include <mfreadwrite.h>

struct H26xFormat
{
    uint16_t width;
    uint16_t height;
    uint8_t framerate;
    H26xProfile profile;
    uint32_t bitrate;
};

struct HookCallbackSocket
{
    SOCKET clientsocket;
    HANDLE clientevent;
};

void CreateSinkWriterPCMToAAC(IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, uint32_t channels, uint32_t samplerate, AACBitrate bitrate, HOOK_SINK_PROC hookproc, void* hookparam);
void CreateSinkWriterNV12ToH26x(IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam);
