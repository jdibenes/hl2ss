
#pragma once

#include "custom_media_sink.h"
#include "custom_media_types.h"
#include "custom_hook_callback.h"
#include <mfreadwrite.h>

struct H26xFormat
{
    uint16_t    width;
    uint16_t    height;
    uint8_t     framerate;
    H26xProfile profile;
    uint32_t    bitrate;
};

struct AACFormat
{
    uint32_t   channels;
    uint32_t   samplerate;
    AACProfile profile;
};

struct HookCallbackSocket
{
    SOCKET clientsocket;
    HANDLE clientevent;
    int    data_profile;
};

void CreateSinkWriterPCMToPCM(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, AACFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam);
void CreateSinkWriterPCMToAAC(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, AACFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam);

void CreateSinkWriterL8ToL8(    CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam);
void CreateSinkWriterNV12ToNV12(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam);
void CreateSinkWriterARGBToARGB(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam);
void CreateSinkWriterNV12ToH26x(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam);
