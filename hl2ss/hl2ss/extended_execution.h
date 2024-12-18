
#pragma once

#include <functional>
#include <stdint.h>

int const INTERFACE_SLOTS = 32;

void ExtendedExecution_Initialize();
void ExtendedExecution_GetApplicationVersion(uint16_t data[4]);
void ExtendedExecution_RunOnMainThread(std::function<void()> f);

void ExtendedExecution_SetFlatMode(bool flat);
bool ExtendedExecution_GetFlatMode();
void ExtendedExecution_SetInterfacePriority(uint32_t id, int32_t priority);
int32_t ExtendedExecution_GetInterfacePriority(uint32_t id);
