
#pragma once

#include "hl2ss_network.h"
#include "custom_sink_writers.h"

void MC_Initialize(HC_Context_Ptr& context, AACFormat format);
void MC_Quit();
void MC_Cleanup();
