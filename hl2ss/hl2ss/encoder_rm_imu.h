
#pragma once

#include "custom_encoder.h"

#include <winrt/Windows.Foundation.Numerics.h>

struct RM_IMU_Metadata
{
    uint64_t timestamp;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

class Encoder_RM_IMU : public CustomEncoder
{
public:
    Encoder_RM_IMU(HOOK_ENCODER_PROC pHookCallback, void* pHookParam);

    void WriteSample(BYTE const* data, uint32_t data_size, LONGLONG timestamp, RM_IMU_Metadata* metadata);
};
