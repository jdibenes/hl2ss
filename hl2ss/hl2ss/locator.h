
#pragma once

#include <Windows.h>

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

void Locator_Initialize();
winrt::Windows::Perception::PerceptionTimestamp Locator_QPCTimestampToPerceptionTimestamp(LONGLONG qpctime);
winrt::Windows::Foundation::Numerics::float4x4 Locator_Locate(winrt::Windows::Perception::PerceptionTimestamp const& timestamp, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world);
winrt::Windows::Foundation::Numerics::float4x4 Locator_Locate(UINT64 qpctime, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world);
winrt::Windows::Perception::Spatial::SpatialCoordinateSystem Locator_GetWorldCoordinateSystem();
