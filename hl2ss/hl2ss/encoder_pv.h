
#pragma once

#include "custom_encoder.h"
#include "types.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Media.Capture.Frames.h>

struct PV_Metadata
{
    winrt::Windows::Foundation::Numerics::float2 f;
    winrt::Windows::Foundation::Numerics::float2 c;
    uint64_t exposure_time;
    uint64x2 exposure_compensation;
    uint32_t lens_position;
    uint32_t focus_state;
    uint32_t iso_speed;
    uint32_t white_balance;
    winrt::Windows::Foundation::Numerics::float2 iso_gains;
    winrt::Windows::Foundation::Numerics::float3 white_balance_gains;
    uint32_t resolution;
    uint64_t timestamp;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

class Encoder_PV : public CustomEncoder
{
public:
    Encoder_PV(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, VideoSubtype subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& options);

    void WriteSample(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame, PV_Metadata* metadata);
};
