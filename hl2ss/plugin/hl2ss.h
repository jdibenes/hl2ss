
#pragma once

#include <stdint.h>

#define UNITY_IMPORT extern "C" __declspec(dllimport)

#define HL2SS_ENABLE_RM  1
#define HL2SS_ENABLE_MC  2
#define HL2SS_ENABLE_PV  4
#define HL2SS_ENABLE_SI  8
#define HL2SS_ENABLE_RC 16

UNITY_IMPORT
void InitializeStreams(uint32_t enable);

UNITY_IMPORT
void DebugMessage(char const* str);
