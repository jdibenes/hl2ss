
#pragma once

#include <winrt/Windows.Perception.Spatial.h>

void InitializeLocator();
winrt::Windows::Perception::Spatial::SpatialCoordinateSystem GetWorldCoordinateSystem();
