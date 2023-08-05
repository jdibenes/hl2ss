
#pragma once

#include "researchmode/ResearchModeApi.h"
#include <WinSock2.h>

#include <winrt/Windows.Perception.Spatial.h>

#include "hl2ss_network.h"
#include "custom_sink_writers.h"

void RM_ZLT_Stream_Mode0(IResearchModeSensor* sensor, z_session_t session, const char* topic_prefix, const bool& should_exit);
void RM_ZLT_Stream_Mode1(IResearchModeSensor* sensor, z_session_t session, const char* topic_prefix, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, const bool& should_exit);
//void RM_ZLT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);

void RM_ZHT_Stream_Mode0(IResearchModeSensor* sensor, z_session_t session, const char* topic_prefix, H26xFormat format, const bool& should_exit);
void RM_ZHT_Stream_Mode1(IResearchModeSensor* sensor, z_session_t session, const char* topic_prefix, H26xFormat format, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, const bool& should_exit);
//void RM_ZHT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);
