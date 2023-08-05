
#pragma once

#include "server.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"

#include <winrt/Windows.Graphics.Imaging.h>

struct ZABFormat
{
    winrt::Windows::Graphics::Imaging::PngFilterMode filter;
};


bool ReceiveAACFormat_Profile(SOCKET clientsocket, AACFormat& profile);
bool ReceiveH26xFormat_Video(SOCKET clientsocket, H26xFormat& format);
bool ReceiveH26xFormat_Divisor(SOCKET clientsocket, H26xFormat& format);
bool ReceiveH26xFormat_Profile(SOCKET clientsocket, H26xFormat& format);
bool ReceiveZABFormat_PNGFilter(SOCKET clientsocket, ZABFormat& format);
