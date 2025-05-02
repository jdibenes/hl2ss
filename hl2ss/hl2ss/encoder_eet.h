
#pragma once

#include "custom_encoder.h"
#include "extended_eye_tracking.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

class Encoder_EET : public CustomEncoder
{
public:
    Encoder_EET(HOOK_ENCODER_PROC pHookCallback, void* pHookParam);

    void WriteSample(winrt::Microsoft::MixedReality::EyeTracking::EyeGazeTrackerReading const& frame, winrt::Windows::Foundation::Numerics::float4x4 const& pose, LONGLONG timestamp);
};
