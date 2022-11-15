
#pragma once

#include <stdint.h>
#include "configuration.h"

#define UNITY_IMPORT extern "C" __declspec(dllimport)

UNITY_IMPORT
void InitializeStreams(uint32_t enable);

UNITY_IMPORT
void DebugMessage(char const* str);
