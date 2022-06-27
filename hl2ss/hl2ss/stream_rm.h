
#pragma once

#include <winrt/Windows.Perception.Spatial.h>

void RM_SetWorldCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world);
void RM_Initialize();
void RM_Quit();
void RM_Cleanup();
