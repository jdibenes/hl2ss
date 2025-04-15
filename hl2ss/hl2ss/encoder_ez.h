
#pragma once

#include <zdepth.hpp>
#include "custom_encoder.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Storage.Streams.h>

struct EZ_Metadata
{
    uint32_t resolution;
    uint32_t _reserved;
    uint64_t timestamp;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

class Encoder_EZ : CustomEncoder
{
private:
    zdepth::DepthCompressor m_compressor;
    uint16_t m_width;
    uint16_t m_height;
    bool m_raw;

public:
    Encoder_EZ(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat);

    bool WriteSample(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame, EZ_Metadata* metadata);
};
