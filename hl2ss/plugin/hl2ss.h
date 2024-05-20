
#pragma once

#include <stdint.h>
#include "configuration.h"

#define UNITY_IMPORT extern "C" __declspec(dllimport)

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

UNITY_IMPORT
void InitializeStreams(uint32_t enable);

UNITY_IMPORT
void InitializeStreamsOnUI(uint32_t enable);

UNITY_IMPORT
void DebugMessage(char const* str);

UNITY_IMPORT
void GetLocalIPv4Address(wchar_t* buffer, int size);

UNITY_IMPORT
int OverrideWorldCoordinateSystem(void* scs_ptr);

//-----------------------------------------------------------------------------
// Message Queue
//-----------------------------------------------------------------------------

UNITY_IMPORT
uint32_t MQ_SI_Peek();

UNITY_IMPORT
void MQ_SI_Pop(uint32_t& command, uint8_t* data);

UNITY_IMPORT
void MQ_SO_Push(uint32_t id);

UNITY_IMPORT
void MQ_Restart();
