
#pragma once

#include <winrt/Windows.Perception.Spatial.h>

void PV_Initialize();
void PV_Quit();
void PV_Cleanup();
void PV_SetWorldFrame(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world);
