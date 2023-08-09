
#pragma once

#include <researchmode/ResearchModeApi.h>
#include <WinSock2.h>

#include <winrt/Windows.Perception.Spatial.h>

void RM_ZHT_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket);
void RM_ZHT_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, winrt::Windows::Perception::Spatial::SpatialLocator const& locator);
void RM_ZHT_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);
