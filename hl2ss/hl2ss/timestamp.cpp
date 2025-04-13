
#include "timestamp.h"
#include "types.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Perception;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static LARGE_INTEGER g_pf;
static UINT64 g_qpc_to_utc_offset = 0;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Timestamp_Initialize()
{
    QueryPerformanceFrequency(&g_pf);
    auto ts = PerceptionTimestampHelper::FromHistoricalTargetTime(winrt::clock::now());
    g_qpc_to_utc_offset = ts.TargetTime().time_since_epoch().count() - ts.SystemRelativeTargetTime().count();
}

// OK
UINT64 Timestamp_GetQPCToUTCOffset()
{
    return g_qpc_to_utc_offset;
}

// OK
UINT64 Timestamp_GetCurrentQPC()
{
    LARGE_INTEGER pc;
    QueryPerformanceCounter(&pc);
    lldiv_t qr = std::div(pc.QuadPart, g_pf.QuadPart);
    return (qr.quot * HNS_BASE) + (qr.rem * HNS_BASE) / g_pf.QuadPart;
}

// OK
UINT64 Timestamp_GetCurrentUTC()
{
    FILETIME ft;
    v64 ts;
    GetSystemTimePreciseAsFileTime(&ft);    
    ts.d.d0.d = ft.dwLowDateTime;
    ts.d.d1.d = ft.dwHighDateTime;
    return ts.q;
}

// OK
TimeSpan Timestamp_U64ToTimeSpan(UINT64 time)
{
    return std::chrono::duration<int64_t, std::ratio<1, HNS_BASE>>(time);
}

// OK
PerceptionTimestamp Timestamp_QPCToPerception(UINT64 qpctime)
{
    return PerceptionTimestampHelper::FromSystemRelativeTargetTime(Timestamp_U64ToTimeSpan(qpctime));
}

// OK
DateTime Timestamp_UTCToDateTime(UINT64 utctime)
{
    return DateTime(Timestamp_U64ToTimeSpan(utctime));
}
