
#pragma once

#include "custom_encoder.h"

struct ANY_Metadata
{
    uint64_t timestamp;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

class Encoder_ANY : public CustomEncoder
{
public:
    Encoder_ANY(HOOK_ENCODER_PROC pHookCallback, void* pHookParam);

    void WriteSample(BYTE const* data, uint32_t data_size, LONGLONG timestamp, ANY_Metadata* metadata);
};
