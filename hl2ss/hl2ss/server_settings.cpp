
#include "server_settings.h"

#include <winrt/Windows.Graphics.Imaging.h>

using namespace winrt::Windows::Graphics::Imaging;

//-----------------------------------------------------------------------------
// Functions 
//-----------------------------------------------------------------------------

// OK
bool ReceiveOperatingMode(SOCKET socket, HANDLE event_error, uint8_t& mode)
{
	bool ok;

	ok = recv_u8(socket, event_error, mode);
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveAACFormat_Profile(SOCKET socket, HANDLE event_error, AACFormat& format)
{
	bool ok;

	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(format.profile));
	if (!ok) { return false; }

	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(format.level));
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
bool ReceiveH26xFormat_Video(SOCKET socket, HANDLE event_error, H26xFormat& format)
{
	bool ok;

	ok = recv_u16(socket, event_error, format.width);
	if (!ok) { return false; }

	ok = recv_u16(socket, event_error, format.height);
	if (!ok) { return false; }

	ok = recv_u8(socket, event_error, format.framerate);
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveH26xFormat_Divisor(SOCKET socket, HANDLE event_error, H26xFormat& format)
{
	bool ok;

	ok = recv_u8(socket, event_error, format.divisor);
	if (!ok) { return false; }

	if (format.divisor <= 0) { return false; }

	return true;
}

// OK
bool ReceiveH26xFormat_Profile(SOCKET socket, HANDLE event_error, H26xFormat& format)
{
	bool ok;

	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(format.profile));
	if (!ok) { return false; }

	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(format.level));
	if (!ok) { return false; }

	ok = recv_u32(socket, event_error, format.bitrate);
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
bool ReceiveEncoderOptions(SOCKET socket, HANDLE event_error, std::vector<uint64_t> &options)
{
	uint8_t count;
	bool ok;	

	ok = recv_u8(socket, event_error, count);
	if (!ok) { return false; }

	options.resize(count * 2);

	ok = recv(socket, event_error, options.data(), static_cast<int>(options.size() * sizeof(uint64_t)));
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveZABFormat_PNGFilter(SOCKET socket, HANDLE event_error, ZABFormat& format)
{
	bool ok;

	ok = recv_u32(socket, event_error, reinterpret_cast<uint32_t&>(format.filter));
	if (!ok) { return false; }

	switch (format.filter)
	{
	case PngFilterMode::Automatic: break;
	case PngFilterMode::None:      break;
	case PngFilterMode::Sub:       break;
	case PngFilterMode::Up:        break;
	case PngFilterMode::Average:   break;
	case PngFilterMode::Paeth:     break;
	case PngFilterMode::Adaptive:  break;
	default: return false;
	}

	return true;
}

// OK
bool ReceiveZABFormat_Profile(SOCKET socket, HANDLE event_error, ZABFormat& format)
{
	bool ok;

	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(format.profile));
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
bool ReceiveMRCVideoOptions(SOCKET socket, HANDLE event_error, MRCVideoOptions& options)
{
	bool ok;

	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(options.enable));
	if (!ok) { return false; }
	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(options.hologram_composition));
	if (!ok) { return false; }
	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(options.recording_indicator));
	if (!ok) { return false; }
	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(options.video_stabilization));
	if (!ok) { return false; }
	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(options.blank_protected));
	if (!ok) { return false; }
	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(options.show_mesh));
	if (!ok) { return false; }
	ok = recv_u8(socket, event_error, reinterpret_cast<uint8_t&>(options.shared));
	if (!ok) { return false; }
	ok = recv_u32(socket, event_error, reinterpret_cast<uint32_t&>(options.global_opacity));
	if (!ok) { return false; }
	ok = recv_u32(socket, event_error, reinterpret_cast<uint32_t&>(options.output_width));
	if (!ok) { return false; }
	ok = recv_u32(socket, event_error, reinterpret_cast<uint32_t&>(options.output_height));
	if (!ok) { return false; }
	ok = recv_u32(socket, event_error, options.video_stabilization_length);
	if (!ok) { return false; }
	ok = recv_u32(socket, event_error, options.hologram_perspective);
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveMRCAudioOptions(SOCKET socket, HANDLE event_error, MRCAudioOptions& options)
{
	bool ok;

	ok = recv_u32(socket, event_error, options.mixer_mode);
	if (!ok) { return false; }
	ok = recv_u32(socket, event_error, reinterpret_cast<uint32_t&>(options.loopback_gain));
	if (!ok) { return false; }
	ok = recv_u32(socket, event_error, reinterpret_cast<uint32_t&>(options.microphone_gain));
	if (!ok) { return false; }

	return true;
}

// OK
bool ReceiveEETFramerate(SOCKET socket, HANDLE event_error, uint8_t& fps)
{
	bool ok;

	ok = recv_u8(socket, event_error, fps);
	if (!ok) { return false; }

	return true;
}
