
#pragma once

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

void ExtendedEyeTracking_Initialize();
void ExtendedEyeTracking_QueryCapabilities();
winrt::Windows::Perception::Spatial::SpatialLocator ExtendedEyeTracking_CreateLocator();
void ExtendedEyeTracking_SetTargetFrameRate(int index);
winrt::Microsoft::MixedReality::EyeTracking::EyeGazeTrackerReading ExtendedEyeTracking_GetReading(winrt::Windows::Foundation::DateTime const& ts, int64_t max_delta);
