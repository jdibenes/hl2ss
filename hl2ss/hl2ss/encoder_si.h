
#pragma once

#include "custom_encoder.h"
#include "spatial_input.h"

#include <winrt/Windows.Perception.People.h>

class Encoder_SI : public CustomEncoder
{
public:
    Encoder_SI(HOOK_ENCODER_PROC pHookCallback, void* pHookParam);

    void WriteSample(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, winrt::Windows::Perception::People::JointPose* left_hand, winrt::Windows::Perception::People::JointPose* right_hand, LONGLONG timestamp);
};
