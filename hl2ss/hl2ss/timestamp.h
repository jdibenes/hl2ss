
#pragma once

#include <Windows.h>

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Perception.h>

uint64_t const HNS_BASE = 10ULL * 1000ULL * 1000ULL;

void Timestamp_Initialize();

UINT64 Timestamp_GetQPCToUTCOffset();
UINT64 Timestamp_GetCurrentQPC();
UINT64 Timestamp_GetCurrentUTC();

winrt::Windows::Foundation::TimeSpan Timestamp_U64ToTimeSpan(UINT64 time);
winrt::Windows::Perception::PerceptionTimestamp Timestamp_QPCToPerception(UINT64 qpctime);
