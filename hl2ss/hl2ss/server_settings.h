
#pragma once

#include "server.h"
#include "custom_sink_writers.h"
#include "custom_video_effect.h"
#include "custom_audio_effect.h"

bool ReceiveOperatingMode(SOCKET socket, HANDLE event_error, uint8_t& mode);
bool ReceiveAACFormat_Profile(SOCKET socket, HANDLE event_error, AACFormat& profile);
bool ReceiveH26xFormat_Video(SOCKET socket, HANDLE event_error, H26xFormat& format);
bool ReceiveH26xFormat_Divisor(SOCKET socket, HANDLE event_error, H26xFormat& format);
bool ReceiveH26xFormat_Profile(SOCKET socket, HANDLE event_error, H26xFormat& format);
bool ReceiveEncoderOptions(SOCKET socket, HANDLE event_error, std::vector<uint64_t>& options);
bool ReceiveZABFormat_PNGFilter(SOCKET socket, HANDLE event_error, ZABFormat& format);
bool ReceiveZABFormat_Profile(SOCKET socket, HANDLE event_error, ZABFormat& format);
bool ReceiveMRCVideoOptions(SOCKET socket, HANDLE event_error, MRCVideoOptions& options);
bool ReceiveMRCAudioOptions(SOCKET socket, HANDLE event_error, MRCAudioOptions& options);
bool ReceiveEETFramerate(SOCKET socket, HANDLE event_error, uint8_t& fps);
