
#pragma once

#include <stdint.h>
#include "configuration.h"

#define UNITY_IMPORT extern "C" __declspec(dllimport)

UNITY_IMPORT
void InitializeStreamsOnUI(uint32_t enable);

UNITY_IMPORT
void DebugMessage(char const* str);

UNITY_IMPORT
void GetLocalIPv4Address(wchar_t* buffer, int size);

UNITY_IMPORT
int OverrideWorldCoordinateSystem(void* scs_ptr);
