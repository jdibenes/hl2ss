
#pragma once

#include <Windows.h>

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.People.h>

int const HAND_JOINTS = 26;

struct SpatialInput_Frame
{
    winrt::Windows::Foundation::Numerics::float3 position;
    winrt::Windows::Foundation::Numerics::float3 forward;
    winrt::Windows::Foundation::Numerics::float3 up;
};

struct SpatialInput_Ray
{
    winrt::Windows::Foundation::Numerics::float3 origin;
    winrt::Windows::Foundation::Numerics::float3 direction;
};

typedef void (*HOOK_SI_PROC)(uint32_t, SpatialInput_Frame*, SpatialInput_Ray*, winrt::Windows::Perception::People::JointPose*, winrt::Windows::Perception::People::JointPose*, UINT64, void*);

bool SpatialInput_WaitForConsent();
void SpatialInput_Startup();
void SpatialInput_Cleanup();
void SpatialInput_ExecuteSensorLoop(HOOK_SI_PROC hook, void* param, HANDLE event_stop);
