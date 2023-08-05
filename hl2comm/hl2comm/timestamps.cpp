
#include <Windows.h>
#include "timestamps.h"
#include "types.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Perception.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Perception;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
UINT64 QPCTickstoQPCTimestamp(LARGE_INTEGER const &pc)
{
	LARGE_INTEGER pf;
	QueryPerformanceFrequency(&pf);
	lldiv_t qr = std::div(pc.QuadPart, pf.QuadPart);
	return (qr.quot * HNS_BASE) + (qr.rem * HNS_BASE) / pf.QuadPart;
}

// OK
UINT64 FTToU64(FILETIME const &ft)
{
	v64 ts;
	ts.d.d0.d = ft.dwLowDateTime;
	ts.d.d1.d = ft.dwHighDateTime;
	return ts.q;
}

// OK
UINT64 GetCurrentQPCTimestamp()
{
	LARGE_INTEGER pc;
	QueryPerformanceCounter(&pc);
	return QPCTickstoQPCTimestamp(pc);
}

// OK
UINT64 GetCurrentUTCTimestamp()
{
	FILETIME ft;	
	GetSystemTimePreciseAsFileTime(&ft);
	return FTToU64(ft);
}

// OK
static DWORD WINAPI ComputeQPCToUTCOffset(void* offset)
{
	LARGE_INTEGER pc;
	FILETIME ft;
	UINT64 qpc;
	UINT64 utc;
	QueryPerformanceCounter(&pc);
	GetSystemTimePreciseAsFileTime(&ft);
	qpc = QPCTickstoQPCTimestamp(pc);
	utc = FTToU64(ft);
	*((u64*)offset) = utc - qpc;
	return 0;
}

// OK
UINT64 GetQPCToUTCOffset()
{
	u64 offset;
	HANDLE h = CreateThread(NULL, 0, ComputeQPCToUTCOffset, &offset, 0, NULL);
	WaitForSingleObject(h, INFINITE);
	CloseHandle(h);
	return offset;
}

// OK
UINT64 GetQPCToUTCOffset(int samples)
{
	u64 offset = GetQPCToUTCOffset();
	u64 next;
	for (int i = 0; i < samples; ++i) { if ((next = GetQPCToUTCOffset()) < offset) { offset = next; } }
	return offset;
}

// OK
PerceptionTimestamp QPCTimestampToPerceptionTimestamp(LONGLONG qpctime)
{
	return PerceptionTimestampHelper::FromSystemRelativeTargetTime(QPCTimestampToTimeSpan(qpctime));
}

// OK
TimeSpan QPCTimestampToTimeSpan(LONGLONG qpctime)
{
	return std::chrono::duration<int64_t, std::ratio<1, HNS_BASE>>(qpctime);
}
