
#include "server.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"
#include "custom_video_effect.h"
#include "ipc_sc.h"

#include <winrt/Windows.Graphics.Imaging.h>

using namespace winrt::Windows::Graphics::Imaging;

//-----------------------------------------------------------------------------
// Functions 
//-----------------------------------------------------------------------------

// OK
bool ReceiveAACFormat_Profile(SOCKET clientsocket, AACFormat& format)
{
	bool ok;

	ok = recv_u8(clientsocket, *(uint8_t*)&format.profile);
	if (!ok) { return false; }

	ok = recv_u8(clientsocket, *(uint8_t*)&format.level);
	if (!ok) { return false; }

	switch (format.profile)
	{
	case AACProfile::AACProfile_12000: break;
	case AACProfile::AACProfile_16000: break;
	case AACProfile::AACProfile_20000: break;
	case AACProfile::AACProfile_24000: break;
	case AACProfile::AACProfile_None:  break;
	default:                           return false;
	}

	switch (format.level)
	{
	case AACLevel::AACLevel_L2:     break;
	case AACLevel::AACLevel_L4:     break;
	case AACLevel::AACLevel_L5:     break;
	case AACLevel::AACLevel_HEv1L2: break;
	case AACLevel::AACLevel_HEv1L4: break;
	case AACLevel::AACLevel_HEv1L5: break;
	case AACLevel::AACLevel_HEv2L2: break;
	case AACLevel::AACLevel_HEv2L3: break;
	case AACLevel::AACLevel_HEv2L4: break;
	case AACLevel::AACLevel_HEv2L5: break;
	default:                        return false;
	}

	return true;
}

// OK
bool ReceiveH26xFormat_Video(SOCKET clientsocket, H26xFormat& format)
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
bool ReceiveH26xFormat_Divisor(SOCKET clientsocket, H26xFormat& format)
{
	bool ok;

	ok = recv_u8(clientsocket, format.divisor);
	if (!ok) { return false; }

	if (format.divisor <= 0) { return false; }

	return true;
}

// OK
bool ReceiveH26xFormat_Profile(SOCKET clientsocket, H26xFormat& format)
{
	bool ok;

	ok = recv_u8(clientsocket, *((uint8_t*)&format.profile));
	if (!ok) { return false; }

	ok = recv_u8(clientsocket, *((uint8_t*)&format.level));
	if (!ok) { return false; }

	ok = recv_u32(clientsocket, format.bitrate);
	if (!ok) { return false; }

	switch (format.profile)
	{
	case H26xProfile::H264Profile_Base: break;
	case H26xProfile::H264Profile_Main: break;
	case H26xProfile::H264Profile_High: break;
	case H26xProfile::H265Profile_Main: break;
	case H26xProfile::H26xProfile_None: break;
	default: return false;
	}

	if (format.bitrate <= 0) { return false; }

	return true;
}

// OK
bool ReceiveH26xEncoder_Options(SOCKET clientsocket, std::vector<uint64_t> &options)
{
	bool ok;
	uint8_t count;

	ok = recv_u8(clientsocket, count);
	if (!ok) { return false; }

	options.resize(count * 2);

	ok = recv(clientsocket, (char*)options.data(), (int)(options.size() * sizeof(uint64_t)));
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveZABFormat_PNGFilter(SOCKET clientsocket, ZABFormat& format)
{
	bool ok;
	uint8_t filterid;

	ok = recv_u8(clientsocket, filterid);
	if (!ok) { return false; }

	switch (filterid)
	{
	case static_cast<int>(PngFilterMode::Automatic): format.filter =  PngFilterMode::Automatic;  break;
	case static_cast<int>(PngFilterMode::None):      format.filter =  PngFilterMode::None;       break;
	case static_cast<int>(PngFilterMode::Sub):       format.filter =  PngFilterMode::Sub;        break;
	case static_cast<int>(PngFilterMode::Up):        format.filter =  PngFilterMode::Up;         break;
	case static_cast<int>(PngFilterMode::Average):   format.filter =  PngFilterMode::Average;    break;
	case static_cast<int>(PngFilterMode::Paeth):     format.filter =  PngFilterMode::Paeth;      break;
	case static_cast<int>(PngFilterMode::Adaptive):  format.filter =  PngFilterMode::Adaptive;   break;
	default: return false;
	}

	return true;
}

// OK
bool ReceiveZABFormat_Profile(SOCKET clientsocket, ZABFormat& format)
{
	bool ok;

	ok = recv_u8(clientsocket, *(uint8_t*)&format.profile);
	if (!ok) { return false; }

	switch (format.profile)
	{
	case ZProfile::ZProfile_Same:   break;
	case ZProfile::ZProfile_Zdepth: break;
	default: return false;
	}

	return true;
}

// OK
bool ReceiveMRCVideoOptions(SOCKET clientsocket, MRCVideoOptions& options)
{
	bool ok;

	ok = recv_u8(clientsocket, *(uint8_t*)&options.enable);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, *(uint8_t*)&options.hologram_composition);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, *(uint8_t*)&options.recording_indicator);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, *(uint8_t*)&options.video_stabilization);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, *(uint8_t*)&options.blank_protected);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, *(uint8_t*)&options.show_mesh);
	if (!ok) { return false; }
	ok = recv_u8(clientsocket, *(uint8_t*)&options.shared);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, *(uint32_t*)&options.global_opacity);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, *(uint32_t*)&options.output_width);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, *(uint32_t*)&options.output_height);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, options.video_stabilization_length);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, options.hologram_perspective);
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveMRCAudioOptions(SOCKET clientsocket, MRCAudioOptions& options)
{
	bool ok;

	ok = recv_u32(clientsocket, options.mixer_mode);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, *(uint32_t*)&options.loopback_gain);
	if (!ok) { return false; }
	ok = recv_u32(clientsocket, *(uint32_t*)&options.microphone_gain);
	if (!ok) { return false; }

	return true;
}
