
#pragma once

#include "custom_encoder.h"

#include <winrt/Windows.Foundation.Numerics.h>

struct RM_VLC_Metadata
{
    uint64_t sensor_ticks;
    uint64_t exposure;
    uint32_t gain;
    uint32_t _reserved;
    uint64_t timestamp;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

struct RM_VLC_VideoTransform
{
    uint32_t chromasize;
    VideoSubtype subtype;
};

class Encoder_RM_VLC : public CustomEncoder
{
private:
	uint32_t m_framebytes;
	uint32_t m_lumasize;
	uint32_t m_chromasize;
	LONGLONG m_duration;

    static RM_VLC_VideoTransform const m_vt_lut[2];

    static RM_VLC_VideoTransform GetTransform(H26xFormat const& format);

public:
	Encoder_RM_VLC(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, std::vector<uint64_t> const& options);

	void WriteSample(BYTE const* pImage, LONGLONG timestamp, RM_VLC_Metadata* metadata);

    static void SetH26xFormat(H26xFormat& format);
};
