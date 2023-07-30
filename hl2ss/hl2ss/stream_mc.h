
#pragma once

#include "hl2ss_network.h"
#include "custom_sink_writers.h"

void MC_Initialize(const char* client_id, z_session_t session, AACFormat format);
void MC_Quit();
void MC_Cleanup();
