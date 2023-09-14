
#pragma once

#include "server.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"
#include "custom_video_effect.h"
#include "custom_audio_effect.h"

#include <winrt/Windows.Graphics.Imaging.h>

enum ZProfile : uint8_t
{
    ZProfile_Same,
    ZProfile_Zdepth
};

struct ZABFormat
{
    winrt::Windows::Graphics::Imaging::PngFilterMode filter;
    ZProfile profile;
    uint8_t _reserved[3];
};

bool ReceiveAACFormat_Profile(SOCKET clientsocket, AACFormat& profile);
bool ReceiveH26xFormat_Video(SOCKET clientsocket, H26xFormat& format);
bool ReceiveH26xFormat_Divisor(SOCKET clientsocket, H26xFormat& format);
bool ReceiveH26xFormat_Profile(SOCKET clientsocket, H26xFormat& format);
bool ReceiveH26xEncoder_Options(SOCKET clientsocket, std::vector<uint64_t>& options);
bool ReceiveZABFormat_PNGFilter(SOCKET clientsocket, ZABFormat& format);
bool ReceiveZABFormat_Profile(SOCKET clientsocket, ZABFormat& format);
bool ReceiveMRCVideoOptions(SOCKET clientsocket, MRCVideoOptions& options);
bool ReceiveMRCAudioOptions(SOCKET clientsocket, MRCAudioOptions& options);
