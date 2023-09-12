
#pragma once

#include <researchmode/ResearchModeApi.h>
#include <WinSock2.h>

#include <winrt/Windows.Perception.Spatial.h>

void RM_ZLT_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket);
void RM_ZLT_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, winrt::Windows::Perception::Spatial::SpatialLocator const& locator);
void RM_ZLT_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);
