
#pragma once

#include <functional>
#include <stdint.h>
#include "types.h"

#include <winrt/Windows.Foundation.h>

int const INTERFACE_SLOTS = 32;

enum Exception : uint32_t
{
    Exception_UnknownNetworkAddress  = bit(0),
    Exception_DisabledResearchMode   = bit(1),
    Exception_AccessDeniedCamera     = bit(2),
    Exception_AccessDeniedMicrophone = bit(3),
    Exception_AccessDeniedEyeTracker = bit(4),
    Exception_AccessDeniedMovements  = bit(5),
};

void ExtendedExecution_Initialize();
void ExtendedExecution_Request();
void ExtendedExecution_GetApplicationVersion(uint16_t data[4]);
void ExtendedExecution_RunOnMainThread(std::function<void()> f);
void ExtendedExecution_EnterException(Exception e);
Exception ExtendedExecution_GetExceptions();
void ExtendedExecution_MessageBox(winrt::hstring message);

void ExtendedExecution_SetFlatMode(bool flat);
bool ExtendedExecution_GetFlatMode();
void ExtendedExecution_SetInterfacePriority(uint32_t id, int32_t priority);
int32_t ExtendedExecution_GetInterfacePriority(uint32_t id);
void ExtendedExecution_SetQuietMode(bool quiet);
bool ExtendedExecution_GetQuietMode();
