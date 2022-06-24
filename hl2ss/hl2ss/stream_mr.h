
#pragma once

#include <winrt/Windows.Perception.Spatial.h>

bool MR_Initialize();
void MR_Update();
winrt::Windows::Perception::Spatial::SpatialCoordinateSystem MR_GetWorldCoordinateSystem();
