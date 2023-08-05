
#pragma once

#ifdef WINDOWS_UWP
#include <winrt/Windows.Perception.h>
#endif

#define HNS_BASE (10ULL * 1000ULL * 1000ULL)

UINT64 QPCTickstoQPCTimestamp(LARGE_INTEGER const& pc);
UINT64 FTToU64(FILETIME const& ft);
UINT64 GetCurrentQPCTimestamp();
UINT64 GetCurrentUTCTimestamp();
UINT64 GetQPCToUTCOffset();
UINT64 GetQPCToUTCOffset(int samples);

#ifdef WINDOWS_UWP
winrt::Windows::Perception::PerceptionTimestamp QPCTimestampToPerceptionTimestamp(LONGLONG qpctime);
winrt::Windows::Foundation::TimeSpan QPCTimestampToTimeSpan(LONGLONG qpctime);
#endif