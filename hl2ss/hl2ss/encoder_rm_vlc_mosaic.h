
#pragma once

#include "custom_encoder.h"

#include <winrt/Windows.Foundation.Numerics.h>

struct RM_VLC_Mosaic_Metadata
{
    uint64_t host_ticks[4];
    uint64_t sensor_ticks[4];
    uint64_t exposure[4];
    uint32_t gain[4];
    uint64_t timestamp;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

struct RM_VLC_Mosaic_VideoTransform
{
    uint32_t chromasize;
    VideoSubtype subtype;
};

class Encoder_RM_VLC_Mosaic : public CustomEncoder
{
private:
    uint32_t m_framebytes;
    uint32_t m_lumasize;
    uint32_t m_chromasize;
    LONGLONG m_duration;

    static RM_VLC_Mosaic_VideoTransform const m_vt_lut[2];

    static RM_VLC_Mosaic_VideoTransform GetTransform(H26xFormat const& format);

public:
    Encoder_RM_VLC_Mosaic(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, std::vector<uint64_t> const& options);

    void WriteSample(BYTE const* const* pImage, int count, LONGLONG timestamp, RM_VLC_Mosaic_Metadata* metadata);

    static void SetH26xFormat(H26xFormat& format, int count);
};
