
#pragma once

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

void Locator_Initialize();
winrt::Windows::Foundation::Numerics::float4x4 Locator_Locate(winrt::Windows::Perception::PerceptionTimestamp const& timestamp, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world);
winrt::Windows::Foundation::Numerics::float4x4 Locator_GetTransformTo(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& src, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& dst);
winrt::Windows::Perception::Spatial::SpatialCoordinateSystem Locator_GetWorldCoordinateSystem();
void Locator_OverrideWorldCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& scs);
winrt::Windows::Perception::Spatial::SpatialCoordinateSystem Locator_SanitizeSpatialCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& scs);
