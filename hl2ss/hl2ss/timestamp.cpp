
#include "timestamp.h"
#include "types.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Perception;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static UINT64 g_utc_to_qpc_offset = 0;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static UINT64 Timestamp_QPCTicksToQPC(LARGE_INTEGER const &pc)
{
	LARGE_INTEGER pf;
	QueryPerformanceFrequency(&pf);
	lldiv_t qr = std::div(pc.QuadPart, pf.QuadPart);
	return (qr.quot * HNS_BASE) + (qr.rem * HNS_BASE) / pf.QuadPart;
}

// OK
static UINT64 Timestamp_FTToU64(FILETIME const &ft)
{
	v64 ts;
	ts.d.d0.d = ft.dwLowDateTime;
	ts.d.d1.d = ft.dwHighDateTime;
	return ts.q;
}

// OK
static DWORD WINAPI Timestamp_ComputeQPCToUTCOffset(void* offset)
{
	LARGE_INTEGER pc;
	FILETIME ft;
	UINT64 qpc;
	UINT64 utc;
	QueryPerformanceCounter(&pc);
	GetSystemTimePreciseAsFileTime(&ft);
	qpc = Timestamp_QPCTicksToQPC(pc);
	utc = Timestamp_FTToU64(ft);
	*((u64*)offset) = utc - qpc;
	return 0;
}

// OK
static UINT64 Timestamp_SampleQPCToUTCOffset()
{
	u64 offset;
	HANDLE h = CreateThread(NULL, 0, Timestamp_ComputeQPCToUTCOffset, &offset, 0, NULL);
	WaitForSingleObject(h, INFINITE);
	CloseHandle(h);
	return offset;
}

// OK
static UINT64 Timestamp_GetQPCToUTCOffset(int samples)
{
	u64 offset = ~0ULL;
	u64 next;
	for (int i = 0; i < samples; ++i) { if ((next = Timestamp_SampleQPCToUTCOffset()) < offset) { offset = next; } }
	return offset;
}

// OK
void Timestamp_Initialize()
{
	g_utc_to_qpc_offset = Timestamp_GetQPCToUTCOffset(32);
}

// OK
UINT64 Timestamp_GetQPCToUTCOffset()
{
	return g_utc_to_qpc_offset;
}

// OK
UINT64 Timestamp_GetCurrentQPC()
{
	LARGE_INTEGER pc;
	QueryPerformanceCounter(&pc);
	return Timestamp_QPCTicksToQPC(pc);
}

// OK
UINT64 Timestamp_GetCurrentUTC()
{
	FILETIME ft;
	GetSystemTimePreciseAsFileTime(&ft);
	return Timestamp_FTToU64(ft);
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
