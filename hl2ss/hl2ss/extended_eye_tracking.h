
#pragma once

#include <Windows.h>

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

typedef void (*HOOK_EET_PROC)(winrt::Microsoft::MixedReality::EyeTracking::EyeGazeTrackerReading const&, UINT64, void*);

bool ExtendedEyeTracking_WaitForConsent();
void ExtendedEyeTracking_Open(bool restricted_mode);
void ExtendedEyeTracking_Close();
bool ExtendedEyeTracking_Status();
void ExtendedEyeTracking_QueryCapabilities();
winrt::Windows::Foundation::Numerics::float4x4 ExtendedEyeTracking_GetNodeWorldPose(UINT64 host_ticks);
bool ExtendedEyeTracking_SetTargetFrameRate(uint8_t fps);
void ExtendedEyeTracking_ExecuteSensorLoop(HOOK_EET_PROC hook, void* param, HANDLE event_stop);
