
#pragma once

#include "zenoh.h"

void MC_Initialize(const char* client_id, z_session_t session);
void MC_Quit();
void MC_Cleanup();
