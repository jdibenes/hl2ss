
#pragma once

#include "server.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"

#include <winrt/Windows.Graphics.Imaging.h>

bool ReceiveAACFormat_Profile(SOCKET clientsocket, AACFormat& profile);
bool ReceiveH26xFormat_Video(SOCKET clientsocket, H26xFormat& format);
bool ReceiveH26xFormat_Divisor(SOCKET clientsocket, H26xFormat& format);
bool ReceiveH26xFormat_Profile(SOCKET clientsocket, H26xFormat& format);
bool ReceivePNGFilter(SOCKET clientsocket, winrt::Windows::Graphics::Imaging::PngFilterMode& filter);
