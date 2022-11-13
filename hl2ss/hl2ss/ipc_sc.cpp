
#include "server.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"
#include "types.h"

#include <winrt/Windows.Graphics.Imaging.h>

using namespace winrt::Windows::Graphics::Imaging;

//-----------------------------------------------------------------------------
// Functions 
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
bool ReceiveVideoFormat(SOCKET clientsocket, H26xFormat& format)
{
	bool ok;

	ok = recv_u16(clientsocket, format.width);
	if (!ok) { return false; }
	ok = recv_u16(clientsocket, format.height);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, format.framerate);
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveVideoH26x(SOCKET clientsocket, H26xFormat& format)
{
	bool ok;
	uint8_t h264profileid;

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

// OK
bool ReceivePNGFilter(SOCKET clientsocket, PngFilterMode& filter)
{
	bool ok;
	u8 filterid;

	ok = recv_u8(clientsocket, filterid);
	if (!ok) { return false; }

	switch (filterid)
	{
	case 0:  filter = PngFilterMode::Automatic; break;
	case 1:  filter = PngFilterMode::None;      break;
	case 2:  filter = PngFilterMode::Sub;       break;
	case 3:  filter = PngFilterMode::Up;        break;
	case 4:  filter = PngFilterMode::Average;   break;
	case 5:  filter = PngFilterMode::Paeth;     break;
	case 6:  filter = PngFilterMode::Adaptive;  break;
	default: return false;
	}

	return true;
}
