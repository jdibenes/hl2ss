#pragma once

#include "hl2ss_network.h"
#include <winrt/Windows.Foundation.Numerics.h>

using namespace winrt::Windows::Foundation::Numerics;

struct EET_Struct {
	uint64_t timestamp;

	float3 position;
	float4 orientation;

	float3 center_origin;
	float3 center_direction;

	float3 left_origin;
	float3 left_direction;

	float3 right_origin;
	float3 right_direction;

	float left_openness;
	float right_openness;

	float vergence_dist;
	uint32_t valid;
};

typedef void(*EETSubscriptionCallback)(const EET_Struct* sample);

void Receive_EET_Initialize(HC_Context_Ptr& context, EETSubscriptionCallback cb, const char* topic);
void Receive_EET_Quit();
void Receive_EET_Cleanup();

