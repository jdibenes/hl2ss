
#include "server.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"

#include <winrt/Windows.Graphics.Imaging.h>

using namespace winrt::Windows::Graphics::Imaging;

//-----------------------------------------------------------------------------
// Functions 
//-----------------------------------------------------------------------------

// OK
bool ReceiveAudioFormatAAC(SOCKET clientsocket, AACProfile& profile)
{
	bool ok;
	uint8_t aacbitrateid;

	ok = recv_u8(clientsocket, aacbitrateid);
	if (!ok) { return false; }

	switch (aacbitrateid)
	{
	case AACProfile::AACProfile_12000: profile = AACProfile::AACProfile_12000; break;
	case AACProfile::AACProfile_16000: profile = AACProfile::AACProfile_16000; break;
	case AACProfile::AACProfile_20000: profile = AACProfile::AACProfile_20000; break;
	case AACProfile::AACProfile_24000: profile = AACProfile::AACProfile_24000; break;
	case AACProfile::AACProfile_None:  profile = AACProfile::AACProfile_None;  break;
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
	case H26xProfile::H264Profile_Base: format.profile = H26xProfile::H264Profile_Base; break;
	case H26xProfile::H264Profile_Main: format.profile = H26xProfile::H264Profile_Main; break;
	case H26xProfile::H264Profile_High: format.profile = H26xProfile::H264Profile_High; break;
	case H26xProfile::H265Profile_Main: format.profile = H26xProfile::H265Profile_Main; break;
	case H26xProfile::H26xProfile_None: format.profile = H26xProfile::H26xProfile_None; break;
	default: return false;
	}

	if (format.bitrate <= 0) { return false; }

	return true;
}

// OK
bool ReceivePNGFilter(SOCKET clientsocket, PngFilterMode& filter)
{
	bool ok;
	uint8_t filterid;

	ok = recv_u8(clientsocket, filterid);
	if (!ok) { return false; }

	switch (filterid)
	{
	case static_cast<int>(PngFilterMode::Automatic): filter =  PngFilterMode::Automatic;  break;
	case static_cast<int>(PngFilterMode::None):      filter =  PngFilterMode::None;       break;
	case static_cast<int>(PngFilterMode::Sub):       filter =  PngFilterMode::Sub;        break;
	case static_cast<int>(PngFilterMode::Up):        filter =  PngFilterMode::Up;         break;
	case static_cast<int>(PngFilterMode::Average):   filter =  PngFilterMode::Average;    break;
	case static_cast<int>(PngFilterMode::Paeth):     filter =  PngFilterMode::Paeth;      break;
	case static_cast<int>(PngFilterMode::Adaptive):  filter =  PngFilterMode::Adaptive;   break;
	//case RAW_PROFILE:                                filter = (PngFilterMode)RAW_PROFILE; break;
	default: return false;
	}

	return true;
}
