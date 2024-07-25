
#pragma once

#include <stdint.h>
#include "plugin.h"

void MQ_Initialize();
void MQ_Quit();
void MQ_Cleanup();

void MQX_Initialize();
void MQX_Quit();
void MQX_Cleanup();

HL2SS_PLUGIN_EXPORT uint32_t MQ_SI_Peek();
HL2SS_PLUGIN_EXPORT void MQ_SI_Pop(uint32_t& command, uint8_t* data);
HL2SS_PLUGIN_EXPORT void MQ_SO_Push(uint32_t id);
HL2SS_PLUGIN_EXPORT void MQ_Restart();

HL2SS_PLUGIN_EXPORT uint32_t MQX_CO_Peek();
HL2SS_PLUGIN_EXPORT void MQX_CO_Pop(uint32_t& id);
HL2SS_PLUGIN_EXPORT void MQX_CI_Push(uint32_t command, uint32_t size, uint8_t const* data);
HL2SS_PLUGIN_EXPORT void MQX_Restart();
