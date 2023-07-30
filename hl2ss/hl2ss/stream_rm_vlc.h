
#pragma once

#include "researchmode/ResearchModeApi.h"
#include <WinSock2.h>

#include <winrt/Windows.Perception.Spatial.h>

#include "hl2ss_network.h"
#include "custom_sink_writers.h"

void RM_VLC_Stream_Mode0(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format);
void RM_VLC_Stream_Mode1(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format, winrt::Windows::Perception::Spatial::SpatialLocator const& locator);
//void RM_VLC_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);
