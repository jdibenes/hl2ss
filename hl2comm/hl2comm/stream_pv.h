
#pragma once

#include "hl2ss_network.h"
#include "custom_sink_writers.h"


void PV_Initialize(HC_Context_Ptr& context, bool enable_location, H26xFormat format);
void PV_Quit();
void PV_Cleanup();
