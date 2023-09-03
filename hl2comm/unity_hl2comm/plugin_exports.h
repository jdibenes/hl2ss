
#pragma once

#include <stdint.h>
#include "configuration.h"
#include "zenoh.h"


#define UNITY_IMPORT extern "C" __declspec(dllimport)
typedef void(*ZenohSubscriptionCallBack)(const char* name, const z_sample_t& Sample);

UNITY_IMPORT
void InitializeStreamsOnUI(const char* cid, const char* zcfg, uint32_t enable);

UNITY_IMPORT
void DebugMessage(char const* str);

UNITY_IMPORT
RegisterRawZSubscriber(const char* name, const char* keyexpr, ZenohSubscriptionCallBack cb);

UNITY_IMPORT
ZSendMessage(const char* keyexpr, uint8_t * buffer, std::size_t buffer_len, z_encoding_prefix_t encoding, bool block);

UNITY_IMPORT
void GetLocalIPv4Address(wchar_t* buffer, int size);

UNITY_IMPORT
int OverrideWorldCoordinateSystem(void* scs_ptr);
