
#pragma once

#include "zenoh.h"

void EET_Initialize(const char* client_id, z_session_t session, uint8_t fps);
void EET_Quit();
void EET_Cleanup();
