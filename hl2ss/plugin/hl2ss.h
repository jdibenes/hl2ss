
#pragma once

#include <stdint.h>

#define HL2SS_PLUGIN_IMPORT extern "C" __declspec(dllimport)

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

HL2SS_PLUGIN_IMPORT
void InitializeStreams(uint32_t enable);

HL2SS_PLUGIN_IMPORT
void InitializeStreamsOnUI(uint32_t enable);

HL2SS_PLUGIN_IMPORT
void DebugMessage(char const* str);

HL2SS_PLUGIN_IMPORT
void GetLocalIPv4Address(wchar_t* buffer, int size);

HL2SS_PLUGIN_IMPORT
int OverrideWorldCoordinateSystem(void* scs_ptr);

HL2SS_PLUGIN_IMPORT
void CheckExceptions();

//-----------------------------------------------------------------------------
// Message Queue
//-----------------------------------------------------------------------------

HL2SS_PLUGIN_IMPORT
uint32_t MQ_SI_Peek();

HL2SS_PLUGIN_IMPORT
void MQ_SI_Pop(uint32_t& command, uint8_t* data);

HL2SS_PLUGIN_IMPORT
void MQ_SO_Push(uint32_t id);

HL2SS_PLUGIN_IMPORT
void MQ_Restart();

HL2SS_PLUGIN_IMPORT
uint32_t MQX_CO_Peek();

HL2SS_PLUGIN_IMPORT
void MQX_CO_Pop(uint32_t& id);

HL2SS_PLUGIN_IMPORT
void MQX_CI_Push(uint32_t command, uint32_t size, uint8_t const* data);

HL2SS_PLUGIN_IMPORT
void MQX_Restart();

//-----------------------------------------------------------------------------
// Lock
//-----------------------------------------------------------------------------

HL2SS_PLUGIN_IMPORT
void NamedMutex_Destroy(void* p);

HL2SS_PLUGIN_IMPORT
void* NamedMutex_Create(wchar_t const* name);

HL2SS_PLUGIN_IMPORT
int NamedMutex_Acquire(void* p, uint32_t timeout);

HL2SS_PLUGIN_IMPORT
int NamedMutex_Release(void* p);

//-----------------------------------------------------------------------------
// PV
//-----------------------------------------------------------------------------

HL2SS_PLUGIN_IMPORT
void PersonalVideo_RegisterNamedMutex(wchar_t const* name);

//-----------------------------------------------------------------------------
// EV
//-----------------------------------------------------------------------------

HL2SS_PLUGIN_IMPORT
void ExtendedVideo_RegisterNamedMutex(wchar_t const* name);
