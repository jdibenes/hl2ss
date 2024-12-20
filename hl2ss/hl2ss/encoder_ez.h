
#pragma once

#include <zdepth.hpp>
#include "custom_encoder.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Storage.Streams.h>

struct EZ_Metadata
{
    uint32_t resolution;
};

class Encoder_EZ
{
private:
    zdepth::DepthCompressor m_compressor;
    HOOK_ENCODER_PROC m_hook;
    void* m_param;
    uint16_t m_width;
    uint16_t m_height;
    bool m_raw;

public:
    Encoder_EZ(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat);

    bool WriteSample(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame, EZ_Metadata* metadata);
};
