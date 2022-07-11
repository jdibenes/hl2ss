
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include "custom_media_sink.h"
#include "utilities.h"
#include "types.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Perception;

//-----------------------------------------------------------------------------
// Remote Configuration
//-----------------------------------------------------------------------------

// OK
bool ReceiveAudioFormatAAC(SOCKET clientsocket, AACBitrate& bitrate)
{
	bool ok;
	uint8_t aacbitrateid;

	ok = recv_u8(clientsocket, aacbitrateid);
	if (!ok) { return false; }

	switch (aacbitrateid)
	{
	case 0:  bitrate = AACBitrate::AACBitrate_12000; break;
	case 1:  bitrate = AACBitrate::AACBitrate_16000; break;
	case 2:  bitrate = AACBitrate::AACBitrate_20000; break;
	case 3:  bitrate = AACBitrate::AACBitrate_24000; break;
	default: return false;
	}

	return true;
}

// OK
bool ReceiveVideoFormatH26x(SOCKET clientsocket, H26xFormat& format)
{
	bool ok;
	uint8_t h264profileid;

	ok = recv_u16(clientsocket, format.width);
	if (!ok) { return false; }
	ok = recv_u16(clientsocket, format.height);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, format.framerate);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, h264profileid);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, format.bitrate);
	if (!ok) { return false; }

	switch (h264profileid)
	{
	case 0:  format.profile = H264Profile_Base; break;
	case 1:  format.profile = H264Profile_Main; break;
	case 2:  format.profile = H264Profile_High; break;
	case 3:  format.profile = H265Profile_Main; break;
	default: return false;
	}

	if (format.bitrate <= 0) { return false; }

	return true;
}

//-----------------------------------------------------------------------------
// Packing
//-----------------------------------------------------------------------------

// OK
void PackUINT16toUINT32(BYTE const* slo16, BYTE const* shi16, BYTE* dst32, int n32ByteVectors)
{
    uint16x8x2_t srcx;
    
    for (int i = 0; i < n32ByteVectors; ++i)
    {
    srcx.val[0] = vld1q_u16(slo16);
    srcx.val[1] = vld1q_u16(shi16);
    vst2q_u16(dst32, srcx);
    slo16 += 16;
    shi16 += 16;
    dst32 += 32;
    }
}

//-----------------------------------------------------------------------------
// Logging 
//-----------------------------------------------------------------------------

// OK
void ShowMessage(const char* format, ...)
{
	char* text;
	int len;
	va_list arg_list;
	va_start(arg_list, format);
	len = _vscprintf(format, arg_list) + 2;
	text = (char*)malloc(len);
	if (!text) { return; }
	vsprintf_s(text, len, format, arg_list);
	va_end(arg_list);
	text[len - 2] = '\n';
	text[len - 1] = '\0';
	OutputDebugStringA(text);
	free(text);
}

// OK
void ShowMessage(const wchar_t* format, ...)
{
	wchar_t* text;
	int len;
	va_list arg_list;
	va_start(arg_list, format);
	len = _vscwprintf(format, arg_list) + 2;
	text = (wchar_t*)malloc(len * sizeof(wchar_t));
	if (!text) { return; }
	vswprintf_s(text, len, format, arg_list);
	va_end(arg_list);
	text[len - 2] = L'\n';
	text[len - 1] = L'\0';
	OutputDebugStringW(text);
	free(text);
}

//-----------------------------------------------------------------------------
// Critical Section 
//-----------------------------------------------------------------------------

// OK
CriticalSection::CriticalSection(void* pcs)
{
	m_pcs = pcs;
	if (m_pcs) { EnterCriticalSection(static_cast<CRITICAL_SECTION*>(m_pcs)); }
}

// OK
CriticalSection::~CriticalSection()
{
	if (m_pcs) { LeaveCriticalSection(static_cast<CRITICAL_SECTION*>(m_pcs)); }
}

//-----------------------------------------------------------------------------
// Time
//-----------------------------------------------------------------------------

// OK
UINT64 GetCurrentQPCTimestamp()
{
	LARGE_INTEGER pc;
	LARGE_INTEGER pf;

	QueryPerformanceCounter(&pc);
	QueryPerformanceFrequency(&pf);

	lldiv_t qr = std::div(pc.QuadPart, pf.QuadPart);

	return (qr.quot * HNS_BASE) + (qr.rem * HNS_BASE) / pf.QuadPart;
}

// OK
PerceptionTimestamp QPCTimestampToPerceptionTimestamp(LONGLONG qpctime)
{
	return PerceptionTimestampHelper::FromSystemRelativeTargetTime(std::chrono::duration<int64_t, std::ratio<1, HNS_BASE>>(qpctime));
}
