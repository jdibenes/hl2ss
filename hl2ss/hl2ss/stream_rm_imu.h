
#pragma once

#include "researchmode/ResearchModeApi.h"
#include <WinSock2.h>

#include <winrt/Windows.Perception.Spatial.h>

#include "zenoh.h"
#include "custom_sink_writers.h"

void RM_ACC_Stream_Mode0(IResearchModeSensor* sensor, z_session_t& session, const char* client_id);
void RM_ACC_Stream_Mode1(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, winrt::Windows::Perception::Spatial::SpatialLocator const& locator);
//void RM_ACC_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);

void RM_GYR_Stream_Mode0(IResearchModeSensor* sensor, z_session_t& session, const char* client_id);
void RM_GYR_Stream_Mode1(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, winrt::Windows::Perception::Spatial::SpatialLocator const& locator);
//void RM_GYR_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);

void RM_MAG_Stream_Mode0(IResearchModeSensor* sensor, z_session_t& session, const char* client_id);
void RM_MAG_Stream_Mode1(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, winrt::Windows::Perception::Spatial::SpatialLocator const& locator);
//void RM_MAG_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket);
