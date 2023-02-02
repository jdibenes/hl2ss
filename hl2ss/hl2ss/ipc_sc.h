
#pragma once

#include "server.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"

#include <winrt/Windows.Graphics.Imaging.h>

bool ReceiveAudioFormatAAC(SOCKET clientsocket, AACProfile& profile);
bool ReceiveVideoFormat(SOCKET clientsocket, H26xFormat& format);
bool ReceiveVideoH26x(SOCKET clientsocket, H26xFormat& format);
bool ReceivePNGFilter(SOCKET clientsocket, winrt::Windows::Graphics::Imaging::PngFilterMode& filter);
