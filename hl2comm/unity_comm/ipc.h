
#pragma once

#include "../hl2comm/hl2ss_network.h"


//Create a callback delegate
typedef void(*ZenohSubscriptionCallBack)(const char* name, const z_sample_t& Sample);


int MQ_SetupZenohRawSubscription(const char* name, const char* keyexpr, ZenohSubscriptionCallBack cb);
bool MQ_SendMessage(const char* keyexpr, uint8_t* buffer, std::size_t buffer_len, z_encoding_prefix_t encoding, bool block = true);

void MQ_Initialize(HC_Context_Ptr& context);
void MQ_Quit();
void MQ_Cleanup();
